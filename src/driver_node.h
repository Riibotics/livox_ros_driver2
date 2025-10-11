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

#ifndef LIVOX_DRIVER_NODE_H
#define LIVOX_DRIVER_NODE_H

#include "include/ros_headers.h"

#include "rii_common_utils/diagnostic_updater.h" 

namespace livox_ros {

class Lddc;

#ifdef BUILDING_ROS1
class DriverNode final : public ros::NodeHandle {
 public:
  DriverNode() = default;
  DriverNode(const DriverNode &) = delete;
  ~DriverNode();
  DriverNode &operator=(const DriverNode &) = delete;

  DriverNode& GetNode() noexcept;

  void PointCloudDataPollThread();
  void ImuDataPollThread();

  std::unique_ptr<Lddc> lddc_ptr_;
  std::shared_ptr<std::thread> pointclouddata_poll_thread_;
  std::shared_ptr<std::thread> imudata_poll_thread_;
  std::shared_future<void> future_;
  std::promise<void> exit_signal_;
};

#elif defined BUILDING_ROS2
class DriverNode final : public rii_common_utils::LifecycleNode {
 public:
  explicit DriverNode(const rclcpp::NodeOptions& options);
  DriverNode(const DriverNode &) = delete;
  ~DriverNode();
  DriverNode &operator=(const DriverNode &) = delete;

  DriverNode& GetNode() noexcept;

  void TickDiagnostic();

 protected:
  rii_common_utils::LifecycleNode::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  rii_common_utils::LifecycleNode::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  rii_common_utils::LifecycleNode::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  rii_common_utils::LifecycleNode::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  rii_common_utils::LifecycleNode::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  void updateLidarStatus(diagnostic_updater::DiagnosticStatusWrapper& status);

 private:
  void PointCloudDataPollThread();
  void ImuDataPollThread();

  std::unique_ptr<Lddc> lddc_ptr_;
  std::shared_ptr<std::thread> pointclouddata_poll_thread_;
  std::shared_ptr<std::thread> imudata_poll_thread_;
  std::shared_future<void> future_;
  std::promise<void> exit_signal_;

  /** Init default system parameter */
  int xfer_format_;
  int multi_topic_ ;
  int data_src_;
  double publish_freq_; /* Hz */
  int output_type_;
  std::string frame_id_;
  uint32_t lidar_handle_ = 0; // LiDAR 핸들을 저장할 변수

  // Diagnostic updater
  std::atomic<int> consecutive_empty_packets_{0};
  std::unique_ptr<rii_common_utils::DiagnosticUpdater> diagnostic_updater_;
};

#endif

} // namespace livox_ros

#endif // LIVOX_DRIVER_NODE_H