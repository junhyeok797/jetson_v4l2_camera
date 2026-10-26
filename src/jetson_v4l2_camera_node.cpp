/**
 * @copyright Chungbuk National University, Autonomous Vehicle Laboratory, 2026. All rights reserved.
 *            Subject to limited distribution and restricted disclosure only.
 *
 * @file      jetson_v4l2_camera_node.cpp
 * @brief     Entry point of jetson_v4l2_camera_node
 *
 * @date      2026-10-24 created by Junhyeok Seo (jun2342@chungbuk.ac.kr)
 */

#include <gst/gst.h>

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "jetson_v4l2_camera/jetson_v4l2_camera.hpp"

int main(int argc, char ** argv)
{
  gst_init(&argc, &argv);
  rclcpp::init(argc, argv);

  int ret = 0;
  try {
    auto node = std::make_shared<jetson_v4l2_camera::JetsonV4l2CameraNode>();
    rclcpp::spin(node);
    ret = node->failed() ? 1 : 0;
    node.reset();  // stop the capture thread before the context goes away
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("jetson_v4l2_camera"), "%s", e.what());
    ret = 1;
  }

  rclcpp::shutdown();
  return ret;
}