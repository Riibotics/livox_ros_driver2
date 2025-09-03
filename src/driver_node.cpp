//
// The MIT License (MIT)
//
// Copyright (c) 2022 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include "driver_node.h"
#include "lddc.h"

namespace livox_ros {

DriverNode& DriverNode::GetNode() noexcept {
  return *this;
}

DriverNode::~DriverNode() {
  lddc_ptr_->lds_->RequestExit();
  exit_signal_.set_value();
  pointclouddata_poll_thread_->join();
  imudata_poll_thread_->join();
}

void DriverNode::TickDiagnostic() {
  if (diagnostic_updater_) {
    diagnostic_updater_->TickFrequencyStatus();
    last_published_steady_clock_ = std::chrono::steady_clock::now();
  }
}

void DriverNode::UpdatePacketStatus(bool is_empty) {
  if (is_empty) {
      consecutive_empty_packets_++;
  } else {
      consecutive_empty_packets_ = 0;
  }
}

void DriverNode::updateLidarStatus(diagnostic_updater::DiagnosticStatusWrapper& status) {
  status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "LiDAR Status OK");

  if (lddc_ptr_ && lddc_ptr_->lds_) {
      status.add("Connected LiDARs", std::to_string(lddc_ptr_->lds_->lidar_count_));
      for (uint32_t i = 0; i < lddc_ptr_->lds_->lidar_count_; ++i) {
          const auto& lidar = lddc_ptr_->lds_->lidars_[i];
          std::string lidar_id = "LiDAR_" + std::to_string(i);
          std::string ip_addr = livox_ros::IpNumToString(lidar.handle);
          std::string state = (lidar.connect_state == kConnectStateSampling) ? "Sampling" : "Connected";
          status.add(lidar_id, "IP: " + ip_addr + ", State: " + state);
      }
  } else {
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "LDS not available");
  }
}

} // namespace livox_ros

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(livox_ros::DriverNode)
