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

#include "lds_lidar.h"

#include <stdio.h>
#include <string.h>
#include <memory>
#include <mutex>
#include <thread>
#include <map>
#include <iostream>
#include <algorithm>
#include <chrono>

#ifdef WIN32
#include <winsock2.h>
#include <ws2def.h>
#pragma comment(lib, "Ws2_32.lib")
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#endif // WIN32

#include "comm/comm.h"
#include "comm/pub_handler.h"

#include "parse_cfg_file/parse_cfg_file.h"
#include "parse_cfg_file/parse_livox_lidar_cfg.h"

#include "call_back/lidar_common_callback.h"
#include "call_back/livox_lidar_callback.h"

using namespace std;

namespace livox_ros {

/** Const varible ------------------------------------------------------------*/
/** For callback use only */
LdsLidar *g_lds_ldiar = nullptr;

std::atomic<int> LdsLidar::sdk_ref_count_{0};

/** Global function for common use -------------------------------------------*/
static std::map<uint32_t, std::chrono::steady_clock::time_point> last_error_log_time;
static std::mutex log_mutex;


/** Lds lidar function -------------------------------------------------------*/
LdsLidar::LdsLidar(double publish_freq)
    : Lds(publish_freq, kSourceRawLidar), 
      auto_connect_mode_(true),
      whitelist_count_(0),
      is_initialized_(false) {
  memset(broadcast_code_whitelist_, 0, sizeof(broadcast_code_whitelist_));
  ResetLdsLidar();
}

LdsLidar::~LdsLidar() {}

void LdsLidar::ResetLdsLidar(void) { ResetLds(kSourceRawLidar); }


bool LdsLidar::InitLdsLidar(const std::string& path_name) {
  if (g_lds_ldiar == nullptr) {
    g_lds_ldiar = this;
  }

  path_ = path_name;

  pub_handler().Init();

  if (!ParseSummaryConfig()) {
    return false;
  }

  if (lidar_summary_info_.lidar_count > 0 && lidar_summary_info_.lidar_count <= kMaxSourceLidar) {
    this->lidar_count_ = lidar_summary_info_.lidar_count;
  } else {
    LivoxLidarConfigParser parser(path_);
    std::vector<UserLivoxLidarConfig> user_configs;
    if (parser.Parse(user_configs) && !user_configs.empty()) {
        this->lidar_count_ = std::min(static_cast<uint8_t>(user_configs.size()), kMaxSourceLidar);
    } else {
        this->lidar_count_ = 0; 
        std::cout << "ERROR: No LiDARs configured in JSON file." << std::endl;
        return false;
    }
  }
  std::cout << "Lidar count is set to: " << static_cast<int>(this->lidar_count_) << std::endl;

  if (!InitLidars()) {
    return false;
  }

  SetLidarPubHandle();
  if (!Start()) {
    return false;
  }
  is_initialized_ = true;
  return true;
}

bool LdsLidar::InitLidars() {
  if (!ParseSummaryConfig()) {
    return false;
  }
  std::cout << "config lidar type: " << static_cast<int>(lidar_summary_info_.lidar_type) << std::endl;

  if (lidar_summary_info_.lidar_type & kLivoxLidarType) {
    if (!InitLivoxLidar()) {
      return false;
    }
  }
  return true;
}

bool LdsLidar::Start() {
  if (lidar_summary_info_.lidar_type & kLivoxLidarType) {
    for (uint32_t i = 0; i < kMaxSourceLidar; ++i) {
      if (lidars_[i].handle != 0 && lidars_[i].lidar_type == kLivoxLidarType) {
        LidarDevice *lidar_device = &lidars_[i];
        const UserLivoxLidarConfig& config = lidar_device->livox_config;
        const uint32_t handle = lidar_device->handle;

        std::cout << "Re-configuring and activating LiDAR, handle: " << handle << std::endl;

        std::lock_guard<std::mutex> lock(config_mutex_);

        if (config.pcl_data_type != -1) {
            SetLivoxLidarPclDataType(handle, static_cast<LivoxLidarPointDataType>(config.pcl_data_type), LivoxLidarCallback::SetDataTypeCallback, this);
        }
        if (config.pattern_mode != -1) {
            SetLivoxLidarScanPattern(handle, static_cast<LivoxLidarScanPattern>(config.pattern_mode), LivoxLidarCallback::SetPatternModeCallback, this);
        }
        if (config.blind_spot_set != -1) {
            SetLivoxLidarBlindSpot(handle, config.blind_spot_set, LivoxLidarCallback::SetBlindSpotCallback, this);
        }
        if (config.dual_emit_en != -1) {
            SetLivoxLidarDualEmit(handle, (config.dual_emit_en != 0), LivoxLidarCallback::SetDualEmitCallback, this);
        }

        LivoxLidarInstallAttitude attitude {
            config.extrinsic_param.roll, config.extrinsic_param.pitch, config.extrinsic_param.yaw,
            config.extrinsic_param.x, config.extrinsic_param.y, config.extrinsic_param.z
        };
        SetLivoxLidarInstallAttitude(handle, &attitude, LivoxLidarCallback::SetAttitudeCallback, this);

        SetLivoxLidarWorkMode(handle, kLivoxLidarNormal, LivoxLidarCallback::WorkModeChangedCallback, nullptr);
        EnableLivoxLidarImuData(handle, LivoxLidarCallback::EnableLivoxLidarImuDataCallback, this);
      }
    }
  }
  return true;
}


bool LdsLidar::ParseSummaryConfig() {
  return ParseCfgFile(path_).ParseSummaryInfo(lidar_summary_info_);
}

bool LdsLidar::InitLivoxLidar() {
  DisableLivoxSdkConsoleLogger();

  // parse user config
  LivoxLidarConfigParser parser(path_);
  std::vector<UserLivoxLidarConfig> user_configs;
  if (!parser.Parse(user_configs)) {
    std::cout << "failed to parse user-defined config" << std::endl;
  }

  // SDK initialization
  if (sdk_ref_count_.fetch_add(1) == 0) {
    if (!LivoxLidarSdkInit(path_.c_str())) {
      std::cout << "Failed to init livox lidar sdk." << std::endl;
      sdk_ref_count_.fetch_sub(1); // Rollback on failure
      return false;
    }
    std::cout << "Livox Lidar SDK Initialized." << std::endl;
  } else {
    std::cout << "Livox Lidar SDK already initialized, ref count: " << sdk_ref_count_.load() << std::endl;
  }

  SetLivoxLidarInfoChangeCallback(LivoxLidarCallback::LidarInfoChangeCallback, g_lds_ldiar);

  // fill in lidar devices
  for (auto& config : user_configs) {
    uint8_t index = 0;
    int8_t ret = g_lds_ldiar->cache_index_.GetFreeIndex(kLivoxLidarType, config.handle, index);
    if (ret != 0) {
      std::cout << "failed to get free index, lidar ip: " << IpNumToString(config.handle) << std::endl;
      continue;
    }
    LidarDevice *p_lidar = &(g_lds_ldiar->lidars_[index]);
    p_lidar->lidar_type = kLivoxLidarType;
    p_lidar->livox_config = config;
    p_lidar->handle = config.handle;

    LidarExtParameter lidar_param;
    lidar_param.handle = config.handle;
    lidar_param.lidar_type = kLivoxLidarType;
    if (config.pcl_data_type == kLivoxLidarCartesianCoordinateLowData) {
      // temporary resolution
      lidar_param.param.roll  = config.extrinsic_param.roll;
      lidar_param.param.pitch = config.extrinsic_param.pitch;
      lidar_param.param.yaw   = config.extrinsic_param.yaw;
      lidar_param.param.x     = config.extrinsic_param.x / 10;
      lidar_param.param.y     = config.extrinsic_param.y / 10;
      lidar_param.param.z     = config.extrinsic_param.z / 10;
    } else {
      lidar_param.param.roll  = config.extrinsic_param.roll;
      lidar_param.param.pitch = config.extrinsic_param.pitch;
      lidar_param.param.yaw   = config.extrinsic_param.yaw;
      lidar_param.param.x     = config.extrinsic_param.x;
      lidar_param.param.y     = config.extrinsic_param.y;
      lidar_param.param.z     = config.extrinsic_param.z;
    }
    pub_handler().AddLidarsExtParam(lidar_param);
  }

  SetLivoxLidarInfoChangeCallback(LivoxLidarCallback::LidarInfoChangeCallback, g_lds_ldiar);
  return true;
}

void LdsLidar::SetLidarPubHandle() {
  pub_handler().SetPointCloudsCallback(LidarCommonCallback::OnLidarPointClounCb, g_lds_ldiar);
  pub_handler().SetImuDataCallback(LidarCommonCallback::LidarImuDataCallback, g_lds_ldiar);

  double publish_freq = Lds::GetLdsFrequency();
  pub_handler().SetPointCloudConfig(publish_freq);
}

bool LdsLidar::LivoxLidarStart() {
  return true;
}

void LdsLidar::Finalize(void) {
  if (sdk_ref_count_.load() > 0) {
    if (sdk_ref_count_.fetch_sub(1) == 1) {
      LivoxLidarSdkUninit();
      printf("Livox Lidar SDK Uninit completely!\n");
    }
    is_initialized_ = false;
  }
}

int LdsLidar::DeInitLdsLidar(void) {
  if (!is_initialized_) {
    printf("LiDAR data source is not initialized, nothing to de-init.\n");
    return 0;
  }
  
  pub_handler().Uninit();
  ResetLdsLidar();
  is_initialized_ = false;

  printf("LdsLidar state cleaned up for reconfiguration. SDK remains initialized.\n");
  return 0;
}

void LdsLidar::StoragePointData(PointFrame* frame) {
  if (frame == nullptr) {
    return;
  }

  uint8_t lidar_number = frame->lidar_num;
  for (uint i = 0; i < lidar_number; ++i) {
    PointPacket& lidar_point = frame->lidar_point[i];
    uint64_t base_time = frame->base_time[i];

    uint8_t index = 0;
    int8_t ret = cache_index_.GetIndex(lidar_point.lidar_type, lidar_point.handle, index);
    if (ret != 0) {
        std::lock_guard<std::mutex> lock(log_mutex);
        auto now = std::chrono::steady_clock::now();
        if (last_error_log_time.find(lidar_point.handle) == last_error_log_time.end() ||
            std::chrono::duration_cast<std::chrono::seconds>(now - last_error_log_time[lidar_point.handle]).count() >= 2) {
            printf("Storage point data failed, can not get index, lidar type:%u, handle:%u.\n", lidar_point.lidar_type, lidar_point.handle);
            last_error_log_time[lidar_point.handle] = now;
        }
        continue;
    }
    PushLidarData(&lidar_point, index, base_time);
  }
}

void LdsLidar::PrepareLidarExit(uint32_t handle) {
  uint8_t index = 0;
  int8_t ret = cache_index_.GetIndex(kLivoxLidarType, handle, index);
  if (ret == 0 && index < kMaxSourceLidar) {
    LidarDevice* lidar = &lidars_[index];
    if (lidar->handle == handle) {
      printf("Preparing to exit for LiDAR handle: %u at index %d\n", handle, index);
      if (lidar->data.storage_packet) {
          lidar->data.rd_idx = 0;
          lidar->data.wr_idx = 0;
      }
      lidar->imu_data.Clear();
      lidar->connect_state = kConnectStateOff;
    }
  } else {
      printf("PrepareLidarExit failed: could not find index for handle %u\n", handle);
  }
}

void LdsLidar::PrepareExit(void) { DeInitLdsLidar(); }

}  // namespace livox_ros