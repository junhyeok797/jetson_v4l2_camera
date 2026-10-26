/**
 * @copyright Chungbuk National University, Autonomous Vehicle Laboratory, 2026. All rights reserved.
 *            Subject to limited distribution and restricted disclosure only.
 *
 * @file      jetson_v4l2_camera.hpp
 * @brief     GMSL2 camera node: V4L2 capture via GStreamer, hardware JPEG, capture-time stamps
 *
 * @date      2026-10-24 created by Junhyeok Seo (jun2342@chungbuk.ac.kr)
 *
 * Timestamps:
 *   v4l2src gives every buffer PTS = (V4L2 buffer timestamp) - (pipeline base time) when the pipeline clock is
 *   the monotonic system clock, so PTS + base time is the kernel capture time (CLOCK_MONOTONIC). It is converted
 *   to UTC with the current CLOCK_REALTIME - CLOCK_MONOTONIC offset (CLOCK_REALTIME is disciplined by phc2sys)
 *   and a per-camera latency offset (measured with the PPS LED test) is subtracted.
 *   PTS survives nvvidconv / nvjpegenc, so the JPEG keeps the capture time.
 */

#ifndef JETSON_V4L2_CAMERA__JETSON_V4L2_CAMERA_HPP_
#define JETSON_V4L2_CAMERA__JETSON_V4L2_CAMERA_HPP_

#include <gst/gst.h>

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>

#include <camera_info_manager/camera_info_manager.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

namespace jetson_v4l2_camera
{

/// Frame counters reported every stats_period
struct CaptureStats
{
  uint64_t frames{0};         ///< published since start
  uint64_t period_frames{0};  ///< published in the current stats period
  uint64_t missing{0};        ///< frames missing according to stamp gaps
  uint64_t non_monotonic{0};  ///< stamps not increasing
  int64_t last_stamp_ns{0};
};

class JetsonV4l2CameraNode : public rclcpp::Node
{
public:
  explicit JetsonV4l2CameraNode(const rclcpp::NodeOptions & i_options = rclcpp::NodeOptions());
  ~JetsonV4l2CameraNode() override;

  /// True if the capture loop stopped because of a pipeline error (process should exit non-zero)
  bool failed() const {return failed_;}

private:
  void declare_parameters();

  std::string build_pipeline() const;
  bool start_pipeline();
  bool check_bus();   ///< Returns false on error / EOS

  rclcpp::Time stamp_of(GstBuffer * i_buffer) const;
  void update_stats(int64_t i_stamp_ns, CaptureStats & o_stats) const;
  void capture_loop();

  void publish_camera_info(const builtin_interfaces::msg::Time & i_stamp);

  // GStreamer
  GstElement * pipeline_{nullptr};
  GstElement * appsink_{nullptr};
  std::atomic<bool> running_{false};
  std::atomic<bool> failed_{false};
  std::thread capture_thread_;

  // ROS
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr p_image_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr p_camera_info_;
  std::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;

  // Parameters
  std::string camera_name_;
  std::string frame_id_;
  std::string device_;
  std::string pixel_format_;
  std::string pipeline_override_;
  std::string timestamp_source_;
  std::string camera_info_url_;
  int width_{};
  int height_{};
  int framerate_{};
  int jpeg_quality_{};
  int64_t latency_offset_ns_{};
  double camera_info_rate_{};
  double stats_period_{};
  int info_every_n_{};
};

}  // namespace jetson_v4l2_camera

#endif  // JETSON_V4L2_CAMERA__JETSON_V4L2_CAMERA_HPP_