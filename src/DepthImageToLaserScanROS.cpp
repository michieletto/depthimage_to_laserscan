/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Author: Chad Rockey
 */

#include <functional>
#include <stdexcept>
#include <string>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rcutils/logging_macros.h>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <depthimage_to_laserscan/DepthImageToLaserScanROS.hpp>

using namespace std::chrono_literals;

namespace depthimage_to_laserscan{

DepthImageToLaserScanROS::DepthImageToLaserScanROS(const rclcpp::NodeOptions & options): rclcpp::Node("depthimage_to_laserscan", options){
  auto qos = rclcpp:: SystemDefaultsQoS();
  cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>("depth_camera_info", qos,
      std::bind(
        &DepthImageToLaserScanROS::infoCb, this,
        std::placeholders::_1));

  depth_image_sub_ =
    this->create_subscription<sensor_msgs::msg::Image>("depth", qos,
      std::bind(&DepthImageToLaserScanROS::depthCb, this, std::placeholders::_1));

  scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", qos);

  this->declare_parameter("scan_time", 0.033);
  this->declare_parameter("range_min", 0.45);
  this->declare_parameter("range_max", 10.0);
  this->declare_parameter("scan_height", 1);
  this->declare_parameter("output_frame", "camera_depth_frame");

  timer_ = this->create_wall_timer(1000ms, std::bind(&DepthImageToLaserScanROS::updateParams, this));

  dtl_ = std::make_unique<depthimage_to_laserscan::DepthImageToLaserScan>(0.033, 0.45, 10.0, 1, "camera_depth_frame");
}

DepthImageToLaserScanROS::~DepthImageToLaserScanROS(){
}

void DepthImageToLaserScanROS::infoCb(sensor_msgs::msg::CameraInfo::SharedPtr info){
  cam_info_ = info;
}

void DepthImageToLaserScanROS::depthCb(const sensor_msgs::msg::Image::SharedPtr image){
  if (nullptr == cam_info_){
    RCLCPP_INFO(get_logger(), "No camera info, skipping point cloud squash");
    return;
  }

  try{
    sensor_msgs::msg::LaserScan::UniquePtr scan_msg = dtl_->convert_msg(image, cam_info_);
    scan_pub_->publish(std::move(scan_msg));
  }
  catch (const std::runtime_error& e){
    RCLCPP_ERROR(get_logger(), "Could not convert depth image to laserscan: %s", e.what());
  }
}

void DepthImageToLaserScanROS::updateParams() {
  float scan_time = 0.0f;
  float range_min = 0.0f;
  float range_max = 0.0f;
  int scan_height = 0;
  std::string output_frame = "test";

  this->get_parameter("scan_time", scan_time);
  this->get_parameter("range_min", range_min);
  this->get_parameter("range_max", range_max);
  this->get_parameter("scan_height", scan_height);
  this->get_parameter("output_frame", output_frame);
  
  dtl_->update_parameters(scan_time, range_min, range_max, scan_height, output_frame);
}

}  // namespace depthimage_to_laserscan

RCLCPP_COMPONENTS_REGISTER_NODE(depthimage_to_laserscan::DepthImageToLaserScanROS)
