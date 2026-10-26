/**
 * @copyright Chungbuk National University, Autonomous Vehicle Laboratory, 2026. All rights reserved.
 *            Subject to limited distribution and restricted disclosure only.
 *
 * @file      jetson_v4l2_camera.cpp
 * @brief     GMSL2 camera node implementation (pipeline, capture loop, stamping, publishing)
 *
 * @date      2026-10-24 created by Junhyeok Seo (jun2342@chungbuk.ac.kr)
 */

#include "jetson_v4l2_camera/jetson_v4l2_camera.hpp"

#include <gst/app/gstappsink.h>

#include <algorithm>
#include <chrono>
#include <ctime>
#include <stdexcept>

namespace jetson_v4l2_camera
{

namespace
{

int64_t to_ns(const timespec & i_ts)
{
  return static_cast<int64_t>(i_ts.tv_sec) * 1000000000LL + i_ts.tv_nsec;
}

// CLOCK_REALTIME - CLOCK_MONOTONIC, sampled now
int64_t realtime_minus_monotonic_ns()
{
  timespec rt{}, mt{};
  clock_gettime(CLOCK_REALTIME, &rt);
  clock_gettime(CLOCK_MONOTONIC, &mt);
  return to_ns(rt) - to_ns(mt);
}

}  // namespace

JetsonV4l2CameraNode::JetsonV4l2CameraNode(const rclcpp::NodeOptions & i_options)
: Node("jetson_v4l2_camera", i_options)
{
  // Parameters
  declare_parameters();

  // Publisher init
  p_image_ = create_publisher<sensor_msgs::msg::CompressedImage>("image/compressed", rclcpp::QoS(10));
  p_camera_info_ = create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", rclcpp::QoS(10));

  // Camera info
  camera_info_manager_ =
    std::make_shared<camera_info_manager::CameraInfoManager>(this, camera_name_, camera_info_url_);
  if (!camera_info_manager_->isCalibrated()) {
    RCLCPP_WARN(get_logger(), "[%s] no calibration loaded (camera_info_url='%s'), publishing size only",
      camera_name_.c_str(), camera_info_url_.c_str());
  }
  info_every_n_ = camera_info_rate_ > 0.0 ?
    std::max(1, static_cast<int>(framerate_ / camera_info_rate_ + 0.5)) : 0;

  // Pipeline and capture thread (the thread owns all appsink reads)
  if (!start_pipeline()) {
    throw std::runtime_error("failed to start GStreamer pipeline");
  }
  running_ = true;
  capture_thread_ = std::thread(&JetsonV4l2CameraNode::capture_loop, this);
}

JetsonV4l2CameraNode::~JetsonV4l2CameraNode()
{
  running_ = false;
  if (capture_thread_.joinable()) {
    capture_thread_.join();
  }
  if (pipeline_) {
    gst_element_set_state(pipeline_, GST_STATE_NULL);
    if (appsink_) {
      gst_object_unref(appsink_);
    }
    gst_object_unref(pipeline_);
  }
}

void JetsonV4l2CameraNode::declare_parameters()
{
  camera_name_ = declare_parameter<std::string>("camera_name", "camera");
  frame_id_ = declare_parameter<std::string>("frame_id", camera_name_ + "_optical_frame");
  device_ = declare_parameter<std::string>("device", "/dev/video0");
  width_ = declare_parameter<int>("width", 1920);
  height_ = declare_parameter<int>("height", 1080);
  framerate_ = declare_parameter<int>("framerate", 30);
  pixel_format_ = declare_parameter<std::string>("pixel_format", "UYVY");
  jpeg_quality_ = declare_parameter<int>("jpeg_quality", 95);
  // Full pipeline override (must end with "appsink name=sink"), e.g. a software pipeline for tests
  pipeline_override_ = declare_parameter<std::string>("pipeline", "");
  // "v4l2": capture time from the V4L2 buffer, "receive": time the JPEG reaches this node (debug only)
  timestamp_source_ = declare_parameter<std::string>("timestamp_source", "v4l2");
  latency_offset_ns_ = declare_parameter<int64_t>("latency_offset_ns", 0);
  camera_info_url_ = declare_parameter<std::string>("camera_info_url", "");
  camera_info_rate_ = declare_parameter<double>("camera_info_rate", 1.0);
  stats_period_ = declare_parameter<double>("stats_period", 10.0);

  if (timestamp_source_ != "v4l2" && timestamp_source_ != "receive") {
    throw std::invalid_argument("timestamp_source must be 'v4l2' or 'receive'");
  }
}

std::string JetsonV4l2CameraNode::build_pipeline() const
{
  if (!pipeline_override_.empty()) {
    return pipeline_override_;
  }
  // do-timestamp=false: keep the driver timestamps. drop=false: never drop silently, gaps are reported instead
  return "v4l2src device=" + device_ + " io-mode=mmap do-timestamp=false"
         " ! video/x-raw,format=" + pixel_format_ + ",width=" + std::to_string(width_) +
         ",height=" + std::to_string(height_) + ",framerate=" + std::to_string(framerate_) + "/1"
         " ! nvvidconv ! video/x-raw(memory:NVMM),format=I420"
         " ! nvjpegenc quality=" + std::to_string(jpeg_quality_) +
         " ! appsink name=sink sync=false max-buffers=8 drop=false";
}

bool JetsonV4l2CameraNode::start_pipeline()
{
  const std::string desc = build_pipeline();
  RCLCPP_INFO(get_logger(), "[%s] pipeline: %s", camera_name_.c_str(), desc.c_str());

  GError * err = nullptr;
  pipeline_ = gst_parse_launch(desc.c_str(), &err);
  if (!pipeline_ || err) {
    RCLCPP_ERROR(get_logger(), "pipeline parse error: %s", err ? err->message : "unknown");
    if (err) {
      g_error_free(err);
    }
    return false;
  }
  appsink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "sink");
  if (!appsink_) {
    RCLCPP_ERROR(get_logger(), "pipeline has no 'appsink name=sink'");
    return false;
  }

  // v4l2src only keeps driver timestamps when the pipeline clock is the monotonic system clock
  GstClock * clock = gst_system_clock_obtain();
  g_object_set(clock, "clock-type", GST_CLOCK_TYPE_MONOTONIC, nullptr);
  gst_pipeline_use_clock(GST_PIPELINE(pipeline_), clock);
  gst_object_unref(clock);

  if (gst_element_set_state(pipeline_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
    RCLCPP_ERROR(get_logger(), "failed to set pipeline to PLAYING");
    check_bus();
    return false;
  }
  return true;
}

bool JetsonV4l2CameraNode::check_bus()
{
  GstBus * bus = gst_element_get_bus(pipeline_);
  bool ok = true;
  while (GstMessage * msg = gst_bus_pop_filtered(bus,
    static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_EOS | GST_MESSAGE_WARNING)))
  {
    if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_EOS) {
      RCLCPP_ERROR(get_logger(), "[%s] end of stream", camera_name_.c_str());
      ok = false;
    } else {
      GError * err = nullptr;
      gchar * dbg = nullptr;
      const bool is_error = GST_MESSAGE_TYPE(msg) == GST_MESSAGE_ERROR;
      if (is_error) {
        gst_message_parse_error(msg, &err, &dbg);
        RCLCPP_ERROR(get_logger(), "[%s] %s (%s)", camera_name_.c_str(), err->message, dbg ? dbg : "");
        ok = false;
      } else {
        gst_message_parse_warning(msg, &err, &dbg);
        RCLCPP_WARN(get_logger(), "[%s] %s (%s)", camera_name_.c_str(), err->message, dbg ? dbg : "");
      }
      g_error_free(err);
      g_free(dbg);
    }
    gst_message_unref(msg);
  }
  gst_object_unref(bus);
  return ok;
}

rclcpp::Time JetsonV4l2CameraNode::stamp_of(GstBuffer * i_buffer) const
{
  const GstClockTime pts = GST_BUFFER_PTS(i_buffer);
  if (timestamp_source_ == "receive" || !GST_CLOCK_TIME_IS_VALID(pts)) {
    return now();
  }
  const int64_t capture_monotonic = static_cast<int64_t>(gst_element_get_base_time(pipeline_) + pts);
  const int64_t utc = capture_monotonic + realtime_minus_monotonic_ns() - latency_offset_ns_;
  return rclcpp::Time(utc, RCL_SYSTEM_TIME);
}

void JetsonV4l2CameraNode::update_stats(int64_t i_stamp_ns, CaptureStats & o_stats) const
{
  const int64_t period_ns = 1000000000LL / std::max(1, framerate_);
  if (o_stats.last_stamp_ns != 0) {
    const int64_t dt = i_stamp_ns - o_stats.last_stamp_ns;
    if (dt <= 0) {
      ++o_stats.non_monotonic;
    } else if (dt > period_ns * 3 / 2) {
      o_stats.missing += static_cast<uint64_t>((dt + period_ns / 2) / period_ns) - 1;
    }
  }
  o_stats.last_stamp_ns = i_stamp_ns;
  ++o_stats.frames;
  ++o_stats.period_frames;
}

void JetsonV4l2CameraNode::capture_loop()
{
  CaptureStats stats;
  auto period_start = std::chrono::steady_clock::now();
  bool pipeline_failed = false;

  while (running_ && rclcpp::ok()) {
    GstSample * sample = gst_app_sink_try_pull_sample(GST_APP_SINK(appsink_), 100 * GST_MSECOND);
    if (!check_bus()) {
      pipeline_failed = true;
      break;
    }
    if (!sample) {
      continue;
    }

    GstBuffer * buffer = gst_sample_get_buffer(sample);
    GstMapInfo map;
    if (buffer && gst_buffer_map(buffer, &map, GST_MAP_READ)) {
      auto msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
      msg->header.stamp = stamp_of(buffer);
      msg->header.frame_id = frame_id_;
      msg->format = "jpeg";
      msg->data.assign(map.data, map.data + map.size);
      gst_buffer_unmap(buffer, &map);

      const bool send_info = info_every_n_ > 0 && stats.frames % static_cast<uint64_t>(info_every_n_) == 0;
      update_stats(rclcpp::Time(msg->header.stamp).nanoseconds(), stats);
      try {
        if (send_info) {
          publish_camera_info(msg->header.stamp);
        }
        p_image_->publish(std::move(msg));
      } catch (const std::exception &) {
        // context already shut down (Ctrl-C) while publishing
        gst_sample_unref(sample);
        break;
      }
    }
    gst_sample_unref(sample);

    const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - period_start).count();
    if (elapsed >= stats_period_) {
      RCLCPP_INFO(get_logger(), "[%s] %.2f fps, frames %lu, missing %lu, non-monotonic stamps %lu",
        camera_name_.c_str(), stats.period_frames / elapsed, stats.frames, stats.missing, stats.non_monotonic);
      stats.period_frames = 0;
      period_start = std::chrono::steady_clock::now();
    }
  }

  if (pipeline_failed) {
    // Exit with an error so the launch file respawns the node
    failed_ = true;
    RCLCPP_FATAL(get_logger(), "[%s] capture stopped, shutting down", camera_name_.c_str());
    rclcpp::shutdown();
  }
}

void JetsonV4l2CameraNode::publish_camera_info(const builtin_interfaces::msg::Time & i_stamp)
{
  auto info = camera_info_manager_->getCameraInfo();
  if (info.width == 0 || info.height == 0) {
    info.width = width_;
    info.height = height_;
  }
  info.header.stamp = i_stamp;
  info.header.frame_id = frame_id_;
  p_camera_info_->publish(info);
}

}  // namespace jetson_v4l2_camera