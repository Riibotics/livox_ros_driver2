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
#include "lds_lidar.h"
#include "parse_cfg_file/parse_livox_lidar_cfg.h"

namespace livox_ros {

#ifdef BUILDING_ROS2

DriverNode::DriverNode(const rclcpp::NodeOptions & node_options)
: rclcpp_lifecycle::LifecycleNode("livox_driver_node", "", node_options)
{
  DRIVER_INFO(*this, "Livox Ros Driver2 Version: %s", LIVOX_ROS_DRIVER2_VERSION_STRING);
  future_ = exit_signal_.get_future();
}

DriverNode::~DriverNode() {
  if (future_.valid() &&
    future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
    exit_signal_.set_value();
  }

  if (pointclouddata_poll_thread_ && pointclouddata_poll_thread_->joinable()) {
    pointclouddata_poll_thread_->join();
  }
  if (imudata_poll_thread_ && imudata_poll_thread_->joinable()) {
    imudata_poll_thread_->join();
  }
}

DriverNode& DriverNode::GetNode() noexcept {
  return *this;
}

DriverNode::CallbackReturn DriverNode::on_configure(const rclcpp_lifecycle::State & /*state*/) {
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
  }

  std::string user_config_path;
  this->get_parameter("user_config_path", user_config_path);
  livox_ros::LivoxLidarConfigParser parser(user_config_path);
  std::vector<livox_ros::UserLivoxLidarConfig> lidar_configs;
  if (parser.Parse(lidar_configs) && !lidar_configs.empty()) {
    lidar_handle_ = lidar_configs[0].handle;
    RCLCPP_INFO(get_logger(), "Node configured for LiDAR handle: %u", lidar_handle_);
  } else {
    RCLCPP_ERROR(get_logger(), "Failed to parse LiDAR config to get handle: %s", user_config_path.c_str());
    return CallbackReturn::FAILURE;
  }

  return CallbackReturn::SUCCESS;
}

DriverNode::CallbackReturn DriverNode::on_activate(const rclcpp_lifecycle::State & /*state*/) {
  lddc_ptr_ = std::make_unique<Lddc>(xfer_format_, multi_topic_, data_src_, output_type_, publish_freq_, frame_id_);
  lddc_ptr_->SetRosNode(this);

  if (data_src_ == kSourceRawLidar) {
    DRIVER_INFO(*this, "Data Source is raw lidar.");
    std::string user_config_path;
    this->get_parameter("user_config_path", user_config_path);
    DRIVER_INFO(*this, "Config file : %s", user_config_path.c_str());

    LdsLidar *read_lidar = LdsLidar::GetInstance(publish_freq_);
    read_lidar->CleanRequestExit();
    lddc_ptr_->RegisterLds(static_cast<Lds *>(read_lidar));

    if ((read_lidar->InitLdsLidar(user_config_path))) {
      DRIVER_INFO(*this, "Init lds lidar success!");
    } else {
      DRIVER_ERROR(*this, "Init lds lidar fail!");
      return CallbackReturn::FAILURE;
    }
  } else {
    DRIVER_ERROR(*this, "Invalid data src (%d), please check the launch file", data_src_);
    return CallbackReturn::FAILURE;
  }

  // Diagnostic updater (period=0.5s)
  diagnostic_updater_ = std::make_shared<diagnostic_updater::Updater>(this, 0.5);
  diagnostic_updater_->setHardwareID("Livox_MID360");

  // Frequency monitoring (matches rii_common_utils: tolerance=0.1, window=max(10, freq))
  min_freq_ptr_ = std::make_shared<double>(publish_freq_);
  max_freq_ptr_ = std::make_shared<double>(publish_freq_);
  diagnostic_updater::FrequencyStatusParam freq_params(
      min_freq_ptr_.get(), max_freq_ptr_.get(), 0.1,
      std::max(10., publish_freq_));
  frequency_status_ = std::make_shared<diagnostic_updater::FrequencyStatus>(freq_params, "frequency");
  diagnostic_updater_->add(*frequency_status_);

  // Working status task
  diagnostic_updater_->add("working_status",
      std::bind(&DriverNode::WorkingStatusUpdaterFunction, this, std::placeholders::_1));

  // Custom LiDAR status task
  diagnostic_updater_->add("LiDAR Status",
      std::bind(&DriverNode::updateLidarStatus, this, std::placeholders::_1));

  if (pointclouddata_poll_thread_ && pointclouddata_poll_thread_->joinable()) {
    pointclouddata_poll_thread_->join();
  }
  if (imudata_poll_thread_ && imudata_poll_thread_->joinable()) {
    imudata_poll_thread_->join();
  }

  exit_signal_ = std::promise<void>();
  future_ = exit_signal_.get_future();

  pointclouddata_poll_thread_ = std::make_shared<std::thread>(&DriverNode::PointCloudDataPollThread, this);
  imudata_poll_thread_ = std::make_shared<std::thread>(&DriverNode::ImuDataPollThread, this);
  return CallbackReturn::SUCCESS;
}

DriverNode::CallbackReturn DriverNode::on_deactivate(const rclcpp_lifecycle::State & /*state*/) {
  if (future_.valid() && future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
    exit_signal_.set_value();
  }

  if (lddc_ptr_ && lddc_ptr_->lds_) {
      lddc_ptr_->lds_->pcd_semaphore_.Signal();
      lddc_ptr_->lds_->imu_semaphore_.Signal();
  }

  RCLCPP_INFO(get_logger(), "Node is deactivated, polling threads signaled to stop.");
  return CallbackReturn::SUCCESS;
}

DriverNode::CallbackReturn DriverNode::on_cleanup(const rclcpp_lifecycle::State & /*state*/) {
  if (pointclouddata_poll_thread_ && pointclouddata_poll_thread_->joinable()) {
    pointclouddata_poll_thread_->join();
    pointclouddata_poll_thread_.reset();
    RCLCPP_INFO(get_logger(), "PointCloud poll thread joined.");
  }
  if (imudata_poll_thread_ && imudata_poll_thread_->joinable()) {
    imudata_poll_thread_->join();
    imudata_poll_thread_.reset();
    RCLCPP_INFO(get_logger(), "IMU poll thread joined.");
  }

  if (lddc_ptr_ && lddc_ptr_->lds_) {
    LdsLidar* read_lidar = static_cast<LdsLidar*>(lddc_ptr_->lds_);
    read_lidar->PrepareLidarExit(lidar_handle_);
    read_lidar->Finalize();
  }
  lddc_ptr_.reset();
  diagnostic_updater_.reset();
  frequency_status_.reset();

  exit_signal_ = std::promise<void>();
  future_ = exit_signal_.get_future();

  RCLCPP_INFO(get_logger(), "Node is cleaned up.");
  return CallbackReturn::SUCCESS;
}

DriverNode::CallbackReturn DriverNode::on_shutdown(const rclcpp_lifecycle::State & /*state*/) {
  RCLCPP_INFO(get_logger(), "Node is shutting down, finalizing SDK.");
  exit_signal_.set_value();

  if (pointclouddata_poll_thread_ && pointclouddata_poll_thread_->joinable()) {
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
  frequency_status_.reset();

  return CallbackReturn::SUCCESS;
}

void DriverNode::TickDiagnostic() {
  if (frequency_status_) {
    frequency_status_->tick();
  }
}

void DriverNode::WorkingStatusUpdaterFunction(diagnostic_updater::DiagnosticStatusWrapper& stat) {
  stat.add("namespace", this->get_namespace());
  DiagStatus status;
  {
    std::lock_guard<std::mutex> lock(working_status_mutex_);
    status = working_status_;
  }
  stat.summary(static_cast<unsigned char>(status.level), status.message);
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

          if (lidar.connect_state.load(std::memory_order_acquire) != kConnectStateSampling) {
              state_str = "Disconnected";
              current_lidar_ok = false;
          } else {
              const uint64_t last_ns = lidar.last_data_ns.load(std::memory_order_relaxed);
              const auto last_tp = std::chrono::steady_clock::time_point(std::chrono::nanoseconds(last_ns));
              auto time_diff = std::chrono::steady_clock::now() - last_tp;
              if (time_diff > std::chrono::seconds(1)) {
                  state_str = "Timeout";
                  current_lidar_ok = false;
              } else if (time_diff > std::chrono::milliseconds(300)) {
                  state_str = "Sampling";
                  RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                       "LiDAR Warning on IP %s: %s", ip_addr.c_str(), state_str.c_str());
              } else {
                  state_str = "Sampling";
              }
          }

          if (!current_lidar_ok) {
              has_error = true;
              RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                    "LiDAR Error on IP %s: %s", ip_addr.c_str(), state_str.c_str());
              {
                std::lock_guard<std::mutex> lock(working_status_mutex_);
                working_status_ = {WorkingStatus::ERROR, "Lidar has a problem"};
              }
          }
          else {
            std::lock_guard<std::mutex> lock(working_status_mutex_);
            working_status_ = {WorkingStatus::OK, "OK"};
          }

          status.add(lidar_id, "IP: " + ip_addr + ", State: " + state_str);
      }
      status.add("Configured LiDARs", std::to_string(configured_lidar_count));

      if (has_error) {
        status.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "One or more LiDARs have an issue.");
      }
  } else {
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "LDS not available");
  }
}

void DriverNode::PointCloudDataPollThread()
{
  std::future_status status;
  std::this_thread::sleep_for(std::chrono::seconds(1));

  while ((status = future_.wait_for(std::chrono::microseconds(0))) == std::future_status::timeout) {
    if (lddc_ptr_) {
        lddc_ptr_->DistributePointCloudData();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

void DriverNode::ImuDataPollThread()
{
  std::future_status status;
  std::this_thread::sleep_for(std::chrono::seconds(3));

  while ((status = future_.wait_for(std::chrono::microseconds(0))) == std::future_status::timeout) {
    if (lddc_ptr_) {
        lddc_ptr_->DistributeImuData();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

#endif // BUILDING_ROS2

} // namespace livox_ros
