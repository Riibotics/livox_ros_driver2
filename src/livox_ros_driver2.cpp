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

#include <iostream>
#include <chrono>
#include <vector>
#include <csignal>
#include <thread>

#include "include/livox_ros_driver2.h"
#include "include/ros_headers.h"
#include "driver_node.h"
#include "lddc.h"
#include "lds_lidar.h"

using namespace livox_ros;

#ifdef BUILDING_ROS1
int main(int argc, char **argv) {
  /** Ros related */
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }

  ros::init(argc, argv, "livox_lidar_publisher");

  // ros::NodeHandle livox_node;
  livox_ros::DriverNode livox_node;

  DRIVER_INFO(livox_node, "Livox Ros Driver2 Version: %s", LIVOX_ROS_DRIVER2_VERSION_STRING);

  /** Init default system parameter */
  int xfer_format = kPointCloud2Msg;
  int multi_topic = 0;
  int data_src = kSourceRawLidar;
  double publish_freq  = 10.0; /* Hz */
  int output_type      = kOutputToRos;
  std::string frame_id = "livox_frame";
  bool lidar_bag = true;
  bool imu_bag   = false;

  livox_node.GetNode().getParam("xfer_format", xfer_format);
  livox_node.GetNode().getParam("multi_topic", multi_topic);
  livox_node.GetNode().getParam("data_src", data_src);
  livox_node.GetNode().getParam("publish_freq", publish_freq);
  livox_node.GetNode().getParam("output_data_type", output_type);
  livox_node.GetNode().getParam("frame_id", frame_id);
  livox_node.GetNode().getParam("enable_lidar_bag", lidar_bag);
  livox_node.GetNode().getParam("enable_imu_bag", imu_bag);

  printf("data source:%u.\n", data_src);

  if (publish_freq > 100.0) {
    publish_freq = 100.0;
  } else if (publish_freq < 0.5) {
    publish_freq = 0.5;
  } else {
    publish_freq = publish_freq;
  }

  livox_node.future_ = livox_node.exit_signal_.get_future();

  /** Lidar data distribute control and lidar data source set */
  livox_node.lddc_ptr_ = std::make_unique<Lddc>(xfer_format, multi_topic, data_src, output_type,
                        publish_freq, frame_id, lidar_bag, imu_bag);
  livox_node.lddc_ptr_->SetRosNode(&livox_node);

  if (data_src == kSourceRawLidar) {
    DRIVER_INFO(livox_node, "Data Source is raw lidar.");

    std::string user_config_path;
    livox_node.getParam("user_config_path", user_config_path);
    DRIVER_INFO(livox_node, "Config file : %s", user_config_path.c_str());

    LdsLidar *read_lidar = LdsLidar::GetInstance(publish_freq);
    livox_node.lddc_ptr_->RegisterLds(static_cast<Lds *>(read_lidar));

    if ((read_lidar->InitLdsLidar(user_config_path))) {
      DRIVER_INFO(livox_node, "Init lds lidar successfully!");
    } else {
      DRIVER_ERROR(livox_node, "Init lds lidar failed!");
    }
  } else {
    DRIVER_ERROR(livox_node, "Invalid data src (%d), please check the launch file", data_src);
  }

  livox_node.pointclouddata_poll_thread_ = std::make_shared<std::thread>(&DriverNode::PointCloudDataPollThread, &livox_node);
  livox_node.imudata_poll_thread_ = std::make_shared<std::thread>(&DriverNode::ImuDataPollThread, &livox_node);
  while (ros::ok()) { usleep(10000); }

  return 0;
}

#elif defined BUILDING_ROS2
namespace livox_ros
{
DriverNode::DriverNode(const rclcpp::NodeOptions & node_options)
: rii_common_utils::LifecycleNode("livox_driver_node", "", node_options)
{
  DRIVER_INFO(*this, "Livox Ros Driver2 Version: %s", LIVOX_ROS_DRIVER2_VERSION_STRING);
  future_ = exit_signal_.get_future();
}
rii_common_utils::LifecycleNode::CallbackReturn DriverNode::on_configure(const rclcpp_lifecycle::State & /*state*/) {
  DeclareAndGetParameter<int>("xfer_format", kPointCloud2Msg, xfer_format_);
  DeclareAndGetParameter<int>("multi_topic", 0, multi_topic_);
  DeclareAndGetParameter<int>("data_src", kSourceRawLidar, data_src_);
  DeclareAndGetParameter<double>("publish_freq", 10.0, publish_freq_);
  DeclareAndGetParameter<int>("output_data_type", kOutputToRos, output_type_);
  DeclareAndGetParameter<std::string>("frame_id", "frame_default", frame_id_);
  if (!this->has_parameter("user_config_path")) this->declare_parameter<std::string>("user_config_path", "path_default");
  if (!this->has_parameter("cmdline_input_bd_code")) this->declare_parameter<std::string>("cmdline_input_bd_code", "000000000000001");
  if (!this->has_parameter("lvx_file_path")) this->declare_parameter<std::string>("lvx_file_path", "/home/livox/livox_test.lvx");

  if (publish_freq_ > 100.0) {
    publish_freq_ = 100.0;
  } else if (publish_freq_ < 0.5) {
    publish_freq_ = 0.5;
  } else {
    publish_freq_ = publish_freq_;
  }

  // future_ = exit_signal_.get_future();

  return rii_common_utils::LifecycleNode::CallbackReturn::SUCCESS;
}

rii_common_utils::LifecycleNode::CallbackReturn DriverNode::on_activate(const rclcpp_lifecycle::State & /*state*/) {
   /** Lidar data distribute control and lidar data source set */
  lddc_ptr_ = std::make_unique<Lddc>(xfer_format_, multi_topic_, data_src_, output_type_, publish_freq_, frame_id_);
  lddc_ptr_->SetRosNode(this);

  if (data_src_ == kSourceRawLidar) {
    DRIVER_INFO(*this, "Data Source is raw lidar.");

    std::string user_config_path;
    this->get_parameter("user_config_path", user_config_path);
    DRIVER_INFO(*this, "Config file : %s", user_config_path.c_str());

    std::string cmdline_bd_code;
    this->get_parameter("cmdline_input_bd_code", cmdline_bd_code);

    LdsLidar *read_lidar = LdsLidar::GetInstance(publish_freq_);

    read_lidar->CleanRequestExit();
    
    lddc_ptr_->RegisterLds(static_cast<Lds *>(read_lidar));

    if ((read_lidar->InitLdsLidar(user_config_path))) {
      DRIVER_INFO(*this, "Init lds lidar success!");
    } else {
      DRIVER_ERROR(*this, "Init lds lidar fail!");
      return rii_common_utils::LifecycleNode::CallbackReturn::FAILURE; 
    }
  } else {
    DRIVER_ERROR(*this, "Invalid data src (%d), please check the launch file", data_src_);
    return rii_common_utils::LifecycleNode::CallbackReturn::FAILURE;
  }

  rii_common_utils::DiagnosticUpdaterBuilder builder(this);
  diagnostic_updater_ = builder.SetPeriodInSec(0.5) // Check every 0.5 sec
                           .SetHardwareID("Livox_MID360")
                           .EnableStatusUpdate()
                           .EnableFrequencyUpdate(publish_freq_)
                           .RegisterCustomUpdaterFunction("LiDAR Status",
                                                          std::bind(&DriverNode::updateLidarStatus, this, std::placeholders::_1))
                           .Build();

  last_published_steady_clock_ = std::chrono::steady_clock::now();


  pointclouddata_poll_thread_ = std::make_shared<std::thread>(&DriverNode::PointCloudDataPollThread, this);
  imudata_poll_thread_ = std::make_shared<std::thread>(&DriverNode::ImuDataPollThread, this);
  return rii_common_utils::LifecycleNode::CallbackReturn::SUCCESS;
}

rii_common_utils::LifecycleNode::CallbackReturn DriverNode::on_deactivate(const rclcpp_lifecycle::State & /*state*/) {
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

  pointclouddata_poll_thread_.reset();
  imudata_poll_thread_.reset();

  return rii_common_utils::LifecycleNode::CallbackReturn::SUCCESS;
}

rii_common_utils::LifecycleNode::CallbackReturn DriverNode::on_cleanup(const rclcpp_lifecycle::State & /*state*/) {

  if (lddc_ptr_ && lddc_ptr_->lds_) {
    LdsLidar* read_lidar = static_cast<LdsLidar*>(lddc_ptr_->lds_);
    read_lidar->DeInitLdsLidar(); // SDK를 종료하지 않는 정리 함수 호출
  }

  lddc_ptr_.reset();
  diagnostic_updater_.reset();

  exit_signal_ = std::promise<void>();
  future_ = exit_signal_.get_future();

  return rii_common_utils::LifecycleNode::CallbackReturn::SUCCESS;
}

rii_common_utils::LifecycleNode::CallbackReturn DriverNode::on_shutdown(const rclcpp_lifecycle::State & /*state*/) {
  RCLCPP_INFO(get_logger(), "Node is shutting down, finalizing SDK.");

  if (pointclouddata_poll_thread_ && pointclouddata_poll_thread_->joinable()) {
    exit_signal_.set_value();
    pointclouddata_poll_thread_->join();
  }
  if (imudata_poll_thread_ && imudata_poll_thread_->joinable()) {
    imudata_poll_thread_->join();
  }

  if (lddc_ptr_ && lddc_ptr_->lds_) {
    LdsLidar* read_lidar = static_cast<LdsLidar*>(lddc_ptr_->lds_);
    read_lidar->Finalize();
  }

  lddc_ptr_.reset();
  diagnostic_updater_.reset();

  return rii_common_utils::LifecycleNode::CallbackReturn::SUCCESS;
}
}  // namespace livox_ros

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(livox_ros::DriverNode)

#endif  // defined BUILDING_ROS2


void DriverNode::PointCloudDataPollThread()
{
  std::future_status status;
  // Wait before start thread
  std::this_thread::sleep_for(std::chrono::seconds(1));
  do {
    if (diagnostic_updater_) {
      bool is_all_ok = true;
      if (lddc_ptr_ && lddc_ptr_->lds_) {
        for (uint32_t i = 0; i < lddc_ptr_->lds_->lidar_count_; ++i) {
          const auto& lidar = lddc_ptr_->lds_->lidars_[i];

          auto time_diff = std::chrono::steady_clock::now() - lidar.last_data_time;
          if (time_diff > std::chrono::seconds(2)) {
            diagnostic_updater_->SetStatusERROR("No data received (timeout)");
            is_all_ok = false;
            break;
          }
        }
      } else {
        diagnostic_updater_->SetStatusERROR("LDS not initialized");
        is_all_ok = false;
      }

      if (!is_all_ok) {
      } else if (consecutive_empty_packets_ >= 3) {
        diagnostic_updater_->SetStatusERROR("Empty packets received consecutively");
      } else {
        diagnostic_updater_->SetStatusOK("LiDAR data streaming");
      }
    }

    lddc_ptr_->DistributePointCloudData();
    status = future_.wait_for(std::chrono::microseconds(0));
  } while (status == std::future_status::timeout);
}

void DriverNode::ImuDataPollThread()
{
  std::future_status status;
  std::this_thread::sleep_for(std::chrono::seconds(3));
  do {
    lddc_ptr_->DistributeImuData();
    status = future_.wait_for(std::chrono::microseconds(0));
  } while (status == std::future_status::timeout);
}





















