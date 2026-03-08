# Livox ROS Driver 2

> This repository is a fork of the [official Livox ROS Driver 2](https://github.com/Livox-SDK/livox_ros_driver2) with additional features for multi-LiDAR deployment and production use.

## Riibotics Extended Features

### LifecycleNode

드라이버 노드가 ROS 2 LifecycleNode로 동작합니다. `on_configure` / `on_activate` / `on_deactivate` / `on_cleanup` / `on_shutdown` 상태 전이를 지원하므로, 런타임에 LiDAR를 안전하게 시작/중지/재설정할 수 있습니다.

`rclcpp_lifecycle::LifecycleNode`을 상속하며, composition 방식의 실행도 지원합니다.

### Sensor ID 기반 토픽 네이밍

`multi_topic`을 활성화한 상태에서 config JSON의 각 LiDAR에 `sensor_id`를 지정하면, IP 기반 토픽 대신 의미 있는 이름의 토픽이 생성됩니다.

**config 예시:**

```json
"lidar_configs" : [
  {
    "sensor_id" : "front",
    "ip" : "192.168.1.210",
    ...
  },
  {
    "sensor_id" : "left",
    "ip" : "192.168.1.211",
    ...
  }
]
```

**결과 토픽:**

| 토픽 | 설명 |
|---|---|
| `/front/livox/lidar` | front LiDAR pointcloud |
| `/front/livox/imu` | front LiDAR IMU |
| `/left/livox/lidar` | left LiDAR pointcloud |
| `/left/livox/imu` | left LiDAR IMU |

`sensor_id`를 생략하면 기존처럼 IP 기반 토픽(`livox/lidar_192_168_1_210`)이 사용됩니다.

`frame_id`도 `sensor_id`에 맞게 자동 설정됩니다 (pointcloud: `front`, IMU: `front_imu`).

### LiDAR 수량 명시 (`lidar_count`)

config JSON의 `lidar_summary_info`에 `lidar_count`를 명시하여 연결할 LiDAR 수를 지정할 수 있습니다. 생략하면 `lidar_configs` 배열 크기로 자동 결정됩니다.

```json
"lidar_summary_info" : {
  "lidar_type": 8,
  "lidar_count": 3
}
```

### Diagnostic 모니터링

`diagnostic_updater`를 통해 각 LiDAR의 연결 상태를 실시간으로 모니터링합니다.

- 각 LiDAR의 연결 상태 (`Sampling` / `Disconnected` / `Timeout`)를 `/diagnostics` 토픽으로 publish
- 데이터 수신이 1초 이상 없으면 `Timeout`, 연결 자체가 안 되면 `Disconnected`로 보고
- `ros2 topic echo /diagnostics` 또는 `rqt_robot_monitor`로 확인 가능

### Composition 지원

`rclcpp_components`로 등록되어 있어 component container에서 로드할 수 있습니다. 메시지 publish 시 zero-copy를 위해 `unique_ptr`로 전달합니다. QoS는 `rmw_qos_profile_sensor_data`를 사용합니다.

---

<details>
<summary><b>Original README (Livox Official)</b></summary>

Livox ROS Driver 2 is the 2nd-generation driver package used to connect LiDAR products produced by Livox, applicable for ROS2 (foxy or humble recommended).

  **Note :**

  As a debugging tool, Livox ROS Driver is not recommended for mass production but limited to test scenarios. You should optimize the code based on the original source to meet your various needs.

## 1. Preparation

### 1.1 OS requirements

  * Ubuntu 18.04 for ROS Melodic;
  * Ubuntu 20.04 for ROS Noetic and ROS2 Foxy;
  * Ubuntu 22.04 for ROS2 Humble;

  **Tips:**

  Colcon is a build tool used in ROS2.

  How to install colcon: [Colcon installation instructions](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)

### 1.2 Install ROS & ROS2

For ROS2 Foxy installation, please refer to:
[ROS Foxy installation instructions](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)

For ROS2 Humble installation, please refer to:
[ROS Humble installation instructions](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

Desktop-Full installation is recommend.

## 2. Build & Run Livox ROS Driver 2

### 2.1 Clone Livox ROS Driver 2 source code:

```shell
git clone https://github.com/Livox-SDK/livox_ros_driver2.git ws_livox/src/livox_ros_driver2
```

  **Note :**

  Be sure to clone the source code in a '[work_space]/src/' folder (as shown above), otherwise compilation errors will occur due to the compilation tool restriction.

### 2.2 Build & install the Livox-SDK2

  **Note :**

  Please follow the guidance of installation in the [Livox-SDK2/README.md](https://github.com/Livox-SDK/Livox-SDK2/blob/master/README.md)

### 2.3 Build the Livox ROS Driver 2:

#### For ROS2 Foxy:
```shell
source /opt/ros/foxy/setup.sh
./build.sh ROS2
```

#### For ROS2 Humble:
```shell
source /opt/ros/humble/setup.sh
./build.sh humble
```

### 2.4 Run Livox ROS Driver 2:

#### For ROS2:
```shell
source ../../install/setup.sh
ros2 launch livox_ros_driver2 [launch file]
```

in which,

* **[launch file]** : is the ROS2 launch file you want to use; the 'launch' folder contains several launch samples for your reference.

A rviz launch example for HAP LiDAR would be:

```shell
ros2 launch livox_ros_driver2 rviz_HAP_launch.py
```

## 3. Launch file and livox_ros_driver2 internal parameter configuration instructions

### 3.1 Launch file configuration instructions

Launch files are in the "ws_livox/src/livox_ros_driver2/launch" directory. Different launch files have different configuration parameter values and are used in different scenarios:

| launch file name          | Description                                                  |
| ------------------------- | ------------------------------------------------------------ |
| rviz_HAP.launch   | Connect to HAP LiDAR device<br>Publish pointcloud2 format  data<br>Autoload rviz |
| msg_HAP.launch     | Connect to HAP LiDAR device<br>Publish livox customized pointcloud data|
| rviz_MID360.launch        | Connect to MID360 LiDAR device<br>Publish pointcloud2 format data <br>Autoload rviz|
| msg_MID360.launch          | Connect to MID360 LiDAR device<br>Publish livox customized pointcloud data |
| rviz_mixed.launch    | Connect to HAP and MID360 LiDAR device<br>Publish pointcloud2 format data <br>Autoload rviz|
| msg_mixed.launch      | Connect to HAP and MID360 LiDAR device<br>Publish livox customized pointcloud data |

### 3.2 Livox ros driver 2 internal main parameter configuration instructions

All internal parameters of Livox_ros_driver2 are in the launch file. Below are detailed descriptions of the three commonly used parameters :

| Parameter    | Detailed description                                         | Default |
| ------------ | ------------------------------------------------------------ | ------- |
| publish_freq | Set the frequency of point cloud publish <br>Floating-point data type, recommended values 5.0, 10.0, 20.0, 50.0, etc. The maximum publish frequency is 100.0 Hz.| 10.0    |
| multi_topic  | If the LiDAR device has an independent topic to publish pointcloud data<br>0 -- All LiDAR devices use the same topic to publish pointcloud data<br>1 -- Each LiDAR device has its own topic to publish point cloud data | 0       |
| xfer_format  | Set pointcloud format<br>0 -- Livox pointcloud2(PointXYZRTLT) pointcloud format<br>1 -- Livox customized pointcloud format<br>2 -- Standard pointcloud2 (pcl :: PointXYZI) pointcloud format in the PCL library (just for ROS) | 0       |

  **Note :**

  Other parameters not mentioned in this table are not suggested to be changed unless fully understood.

&ensp;&ensp;&ensp;&ensp;***Livox_ros_driver2 pointcloud data detailed description :***

1. Livox pointcloud2 (PointXYZRTLT) point cloud format, as follows :

```c
float32 x               # X axis, unit:m
float32 y               # Y axis, unit:m
float32 z               # Z axis, unit:m
float32 intensity       # the value is reflectivity, 0.0~255.0
uint8   tag             # livox tag
uint8   line            # laser number in lidar
float64 timestamp       # Timestamp of point
```
  **Note :**

  The number of points in the frame may be different, but each point provides a timestamp.

2. Livox customized data package format, as follows :

```c
std_msgs/Header header     # ROS standard message header
uint64          timebase   # The time of first point
uint32          point_num  # Total number of pointclouds
uint8           lidar_id   # Lidar device id number
uint8[3]        rsvd       # Reserved use
CustomPoint[]   points     # Pointcloud data
```

&ensp;&ensp;&ensp;&ensp;Customized Point Cloud (CustomPoint) format in the above customized data package :

```c
uint32  offset_time     # offset time relative to the base time
float32 x               # X axis, unit:m
float32 y               # Y axis, unit:m
float32 z               # Z axis, unit:m
uint8   reflectivity    # reflectivity, 0~255
uint8   tag             # livox tag
uint8   line            # laser number in lidar
```

3. The standard pointcloud2 (pcl :: PointXYZI) format in the PCL library (only ROS can publish):

&ensp;&ensp;&ensp;&ensp;Please refer to the pcl :: PointXYZI data structure in the point_types.hpp file of the PCL library.

## 4. LiDAR config

LiDAR Configurations (such as ip, port, data type... etc.) can be set via a json-style config file. Config files for single HAP, Mid360 and mixed-LiDARs are in the "config" folder. The parameter naming *'user_config_path'* in launch files indicates such json file path.

1. Follow is a configuration example for HAP LiDAR (located in config/HAP_config.json):

```json
{
  "lidar_summary_info" : {
    "lidar_type": 8  # protocol type index, please don't revise this value
  },
  "HAP": {
    "device_type" : "HAP",
    "lidar_ipaddr": "",
    "lidar_net_info" : {
      "cmd_data_port": 56000,  # command port
      "push_msg_port": 0,
      "point_data_port": 57000,
      "imu_data_port": 58000,
      "log_data_port": 59000
    },
    "host_net_info" : {
      "cmd_data_ip" : "192.168.1.5",  # host ip (it can be revised)
      "cmd_data_port": 56000,
      "push_msg_ip": "",
      "push_msg_port": 0,
      "point_data_ip": "192.168.1.5",  # host ip
      "point_data_port": 57000,
      "imu_data_ip" : "192.168.1.5",  # host ip
      "imu_data_port": 58000,
      "log_data_ip" : "",
      "log_data_port": 59000
    }
  },
  "lidar_configs" : [
    {
      "ip" : "192.168.1.100",  # ip of the LiDAR you want to config
      "pcl_data_type" : 1,
      "pattern_mode" : 0,
      "blind_spot_set" : 50,
      "extrinsic_parameter" : {
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": 0.0,
        "x": 0,
        "y": 0,
        "z": 0
      }
    }
  ]
}
```

The parameter attributes in the above json file are described in the following table :

**LiDAR configuration parameter**
| Parameter                  | Type    | Description                                                  | Default         |
| :------------------------- | ------- | ------------------------------------------------------------ | --------------- |
| ip             | String  | Ip of the LiDAR you want to config | 192.168.1.100 |
| pcl_data_type             | Int | Choose the resolution of the point cloud data to send<br>1 -- Cartesian coordinate data (32 bits)<br>2 -- Cartesian coordinate data (16 bits) <br>3 --Spherical coordinate data| 1           |
| pattern_mode                | Int     | Space scan pattern<br>0 -- non-repeating scanning pattern mode<br>1 -- repeating scanning pattern mode <br>2 -- repeating scanning pattern mode (low scanning rate) | 0               |
| blind_spot_set (Only for HAP LiDAR)                 | Int     | Set blind spot<br>Range from 50 cm to 200 cm               | 50               |
| extrinsic_parameter |      | Set extrinsic parameter<br> The data types of "roll" "picth" "yaw" are float <br>  The data types of "x" "y" "z" are int<br>               |

For more infomation about the HAP config, please refer to:
[HAP Config File Description](https://github.com/Livox-SDK/Livox-SDK2/wiki/hap-config-file-description)

2. When connecting multiple LiDARs, add objects corresponding to different LiDARs to the "lidar_configs" array. Examples of mixed-LiDARs config file contents are as follows :

```json
{
  "lidar_summary_info" : {
    "lidar_type": 8  # protocol type index, please don't revise this value
  },
  "HAP": {
    "lidar_net_info" : {  # HAP ports, please don't revise these values
      "cmd_data_port": 56000,  # HAP command port
      "push_msg_port": 0,
      "point_data_port": 57000,
      "imu_data_port": 58000,
      "log_data_port": 59000
    },
    "host_net_info" : {
      "cmd_data_ip" : "192.168.1.5",  # host ip
      "cmd_data_port": 56000,
      "push_msg_ip": "",
      "push_msg_port": 0,
      "point_data_ip": "192.168.1.5",  # host ip
      "point_data_port": 57000,
      "imu_data_ip" : "192.168.1.5",  # host ip
      "imu_data_port": 58000,
      "log_data_ip" : "",
      "log_data_port": 59000
    }
  },
  "MID360": {
    "lidar_net_info" : {  # Mid360 ports, please don't revise these values
      "cmd_data_port": 56100,  # Mid360 command port
      "push_msg_port": 56200,
      "point_data_port": 56300,
      "imu_data_port": 56400,
      "log_data_port": 56500
    },
    "host_net_info" : {
      "cmd_data_ip" : "192.168.1.5",  # host ip
      "cmd_data_port": 56101,
      "push_msg_ip": "192.168.1.5",  # host ip
      "push_msg_port": 56201,
      "point_data_ip": "192.168.1.5",  # host ip
      "point_data_port": 56301,
      "imu_data_ip" : "192.168.1.5",  # host ip
      "imu_data_port": 56401,
      "log_data_ip" : "",
      "log_data_port": 56501
    }
  },
  "lidar_configs" : [
    {
      "ip" : "192.168.1.100",  # ip of the HAP you want to config
      "pcl_data_type" : 1,
      "pattern_mode" : 0,
      "blind_spot_set" : 50,
      "extrinsic_parameter" : {
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": 0.0,
        "x": 0,
        "y": 0,
        "z": 0
      }
    },
    {
      "ip" : "192.168.1.12",  # ip of the Mid360 you want to config
      "pcl_data_type" : 1,
      "pattern_mode" : 0,
      "extrinsic_parameter" : {
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": 0.0,
        "x": 0,
        "y": 0,
        "z": 0
      }
    }
  ]
}
```

## 5. Supported LiDAR list

* HAP
* Mid360
* (more types are comming soon...)

## 6. FAQ

### 6.1 launch with "livox_lidar_rviz_HAP.launch" but no point cloud display on the grid?

Please check the "Global Options - Fixed Frame" field in the RViz "Display" pannel. Set the field value to "livox_frame" and check the "PointCloud2" option in the pannel.

### 6.2 launch with command "ros2 launch livox_lidar_rviz_HAP_launch.py" but cannot open shared object file "liblivox_sdk_shared.so" ?

Please add '/usr/local/lib' to the env LD_LIBRARY_PATH.

* If you want to add to current terminal:

  ```shell
  export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib
  ```

* If you want to add to current user:

  ```shell
  vim ~/.bashrc
  export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib
  source ~/.bashrc
  ```

</details>
