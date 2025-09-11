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
  if (lddc_ptr_ && lddc_ptr_->lds_) {
    lddc_ptr_->lds_->RequestExit();
  }
  exit_signal_.set_value();

  if (pointclouddata_poll_thread_ && pointclouddata_poll_thread_->joinable()) {
    pointclouddata_poll_thread_->join();
  }
  if (imudata_poll_thread_ && imudata_poll_thread_->joinable()) {
    imudata_poll_thread_->join();
  }
}

void DriverNode::TickDiagnostic() {
  if (diagnostic_updater_) {
    // diagnostic_updater_->TickFrequencyStatus(); // Considering Livox's frequency oscillation, disable this.
    // @TODO(Riibotics) : How to handle frequency oscillation?
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
  status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "All LiDARs are streaming");

  if (lddc_ptr_ && lddc_ptr_->lds_) {
      int configured_lidar_count = 0;
      bool has_error = false;

      for (uint32_t i = 0; i < kMaxSourceLidar; ++i) {
          const auto& lidar = lddc_ptr_->lds_->lidars_[i];

          if (lidar.handle == 0) continue;

          configured_lidar_count++;
          std::string lidar_id = "LiDAR_" + std::to_string(i) + 
                                 (lidar.livox_config.sensor_id.empty() ? "" : "_" + lidar.livox_config.sensor_id);
          std::string ip_addr = livox_ros::IpNumToString(lidar.handle);
          std::string state_str;
          bool current_lidar_ok = true;

          if (lidar.connect_state != kConnectStateSampling) {
              state_str = "Disconnected";
              current_lidar_ok = false;
          } else {
              auto time_diff = std::chrono::steady_clock::now() - lidar.last_data_time;
              if (time_diff > std::chrono::seconds(2)) {
                  state_str = "Timeout";
                  current_lidar_ok = false;
              } else {
                  state_str = "Sampling";
              }
          }

          if (!current_lidar_ok) {
              has_error = true;
              RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                    "LiDAR Error on IP %s: %s", ip_addr.c_str(), state_str.c_str());
          }

          status.add(lidar_id, "IP: " + ip_addr + ", State: " + state_str);
      }
      status.add("Configured LiDARs", std::to_string(configured_lidar_count));

      if (has_error) {
        status.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "One or more LiDARs have an issue.");
      }

  } else {
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "LDS not available");
  }
}

} // namespace livox_ros

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(livox_ros::DriverNode)
