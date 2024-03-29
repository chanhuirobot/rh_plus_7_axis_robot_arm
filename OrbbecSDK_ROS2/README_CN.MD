# orbbec_camera

---
OrbbecSDK ROS2是用于Orbbec 3D相机的ROS2 wrapper ，可与ROS2环境无缝集成，并支持ROS2 Foxy、Galactic和Humble发行版。## 安装指南

安装 ROS2

- 请参考官方
  [ROS2 安装指南](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

> 如果你的ROS2 命令不能自动补全，把下面两行代码放在你的 `.bashrc`
> 或者 `.zshrc`

```bash
eval "$(register-python-argcomplete3 ros2)"
eval "$(register-python-argcomplete3 colcon)"
```

创建 `colcon` 工作空间

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

获取源代码

```bash
cd ~/ros2_ws/src
git clone https://github.com/orbbec/OrbbecSDK_ROS2.git
```

安装deb依赖

```bash
# assume you have sourced ROS environment, same blow
sudo apt install libgflags-dev nlohmann-json3-dev libgoogle-glog-dev \
ros-$ROS_DISTRO-image-transport ros-$ROS_DISTRO-image-publisher ros-$ROS_DISTRO-camera-info-manager
```

安装 udev rules.

```bash
cd ~/ros2_ws/src/OrbbecSDK_ROS2/orbbec_camera/scripts
sudo bash install_udev_rules.sh
sudo udevadm control --reload-rules && sudo udevadm trigger
```

## 入门指南

```bash
cd ~/ros2_ws/
# build release, Default is Debug
colcon build --event-handlers  console_direct+  --cmake-args  -DCMAKE_BUILD_TYPE=Release
```

启动相机。

- 在终端 1。

```bash
. ./install/setup.bash 
ros2 launch orbbec_camera astra.launch.py # or other launch file, see below table
```

- 在终端 2。

```bash
. ./install/setup.bash 
rviz2
```

选择你想展示的topic。

- 列出 topics / services/ parameters (新开一个终端)。

```bash
ros2 topic list
ros2 service list
ros2 param list
```

- 显示深度到彩色的外参。

```bash
ros2 topic echo --qos-durability=transient_local /camera/extrinsic/depth_to_color  --qos-profile=services_default
```

- 获取设备信息

```bash
ros2 service call /camera/get_device_info orbbec_camera_msgs/srv/GetDeviceInfo '{}'
```

- 查看SDK版本

```bash
ros2 service call /camera/get_sdk_version orbbec_camera_msgs/srv/GetString '{}'

```

- 查看曝光值

```bash
ros2 service call /camera/get_color_exposure orbbec_camera_msgs/srv/GetInt32 '{}'
```

> 如果你需要查看 `ir` 或者 `depth`的曝光, 请把 `/camera/get_color_exposure`
> 改成 `/camera/get_ir_exposure` 或者 `/camera/get_depth_exposure`,下同.

- 查看增益

```bash
ros2 service call /camera/get_color_gain orbbec_camera_msgs/srv/GetInt32 '{}'
```

- 查看白平衡

```bash
ros2 service call /camera/get_white_balance orbbec_camera_msgs/srv/GetInt32 '{}'
```

- 设置自动曝光

```bash
ros2 service call /camera/set_color_auto_exposure std_srvs/srv/SetBool '{data: false}'
```

- 设置白平衡

```bash
ros2 service call /camera/set_white_balance orbbec_camera_msgs/srv/SetInt32 '{data: 4600}'
```

- 开关激光

```bash
ros2 service call  /camera/set_laser_enable std_srvs/srv/SetBool "{data: true}" 
```

- 开关senseor

```bash
ros2 service call /camera/toggle_ir std_srvs/srv/SetBool "{data : true}"

```

- 保存点云

```bash
ros2 service call /camera/save_point_cloud std_srvs/srv/Empty "{}"
```

### 所有相机控制的service

下面的这些服务的名字已经表明了它的功能，但是要注意的是只有当在launch文件里面把相关数据流打开的时候才能用。

- `/camera/get_auto_white_balance`
- `/camera/get_color_exposure`
- `/camera/get_color_gain`
- `/camera/get_depth_exposure`
- `/camera/get_depth_gain`
- `/camera/get_device_info`
- `/camera/get_ir_exposure`
- `/camera/get_ir_gain`
- `/camera/get_ldp_status`
- `/camera/get_sdk_version`
- `/camera/get_white_balance`
- `/camera/set_auto_white_balance`
- `/camera/set_color_auto_exposure`
- `/camera/set_color_exposure`
- `/camera/set_color_gain`
- `/camera/set_depth_auto_exposure`
- `/camera/set_depth_exposure`
- `/camera/set_depth_gain`
- `/camera/set_fan_work_mode`
- `/camera/set_floor_enable`
- `/camera/set_ir_auto_exposure`
- `/camera/set_ir_exposure`
- `/camera/set_ir_gain`
- `/camera/set_laser_enable`
- `/camera/set_ldp_enable`
- `/camera/set_white_balance`
- `/camera/toggle_color`
- `/camera/toggle_depth`
- `/camera/toggle_ir`

### 所有的topic

- `/camera/color/camera_info` : 彩色相机信息。
- `/camera/color/image_raw`: 彩色数据流图像。
- `/camera/depth/camera_info`: 深度相机信息。
- `/camera/depth/image_raw`: 深度相机数据流图像。
- `/camera/depth/points` : 点云，只有  `enable_point_cloud` 设置为 `true`才可用。
- `/camera/depth_registered/points`: 彩色点云  `enable_colored_point_cloud` 还有 `depth_registration`
  都设置为 `true`才可用。
- `/camera/ir/camera_info`:  IR相机信息。
- `/camera/ir/image_raw`: 红外数据流图像。
- `/camera/accel/sample`: 加速度数据流，`enable_sync_output_accel_gyro`配置关闭，`enable_accel`配置打开
- `/camera/gyro/sample`: 陀螺仪数据流，enable_sync_output_accel_gyro`配置关闭，`enable_gyro`配置打开
- `camera/gyro_accel/sample`: 加速度和陀螺仪同步数据流，通过`enable_sync_output_accel_gyro`配置打开

### 多相机

- 首先需要获取所有相机的USB Port, 插入所有的相机在终端执行以下命令:

```bash
ros2 run orbbec_camera list_devices_node
```

- 把参数 `device_num` 设置成为你的相机数量。
- 编辑 `OrbbecSDK_ROS2/launch/multi_xxx.launch.py` 修改相机的`usb_port`。
- 注意不要忘记在`include` 加上 `group` 标签，不然前面的相机的参数会污染后面的相机的参数。

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Node configuration
    cleanup_node = Node(
        package='orbbec_camera',
        executable='ob_cleanup_shm_node',
        name='camera',
        output='screen'
    )

    # Include launch files
    package_dir = get_package_share_directory('orbbec_camera')
    launch_file_dir = os.path.join(package_dir, 'launch')
    launch1_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'gemini2.launch.py')
        ),
        launch_arguments={
            'camera_name': 'camera_01',
            'usb_port': '6-2.4.4.2',  # replace your usb port here 
            'device_num': '2'
        }.items()
    )

    launch2_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'gemini2.launch.py')
        ),
        launch_arguments={
            'camera_name': 'camera_02',
            'usb_port': '6-2.4.1',  # replace your usb port here
            'device_num': '2'
        }.items()
    )

    # If you need more cameras, just add more launch_include here, and change the usb_port and device_num

    # Launch description
    ld = LaunchDescription([
        cleanup_node,
        GroupAction([launch1_include]),
        GroupAction([launch2_include]),
    ])

    return ld

```

- 请注意，Astra相机使用信号量进行进程同步。如果相机启动失败，信号量文件可能会留在`/dev/shm`
  目录下，导致下一次启动卡住。为了避免这种情况，请在启动前运行以下命令：

```bash
ros2 run orbbec_camera ob_cleanup_shm_node
```

这个命令会清理掉`/dev/shm/`下面的文件

- 执行下面的命令启动多相机:

```bash
ros2 launch orbbec_camera multi_camera.launch.py
```

## 调用硬件解码JPEG

### rockchip 和 Amlogic

依赖  `rockchip-mpp-dev` 和 `rockchip-rga-dev`, 不是所有的系统都有这两个包， 名字可能不一样， 请自行搜索。
打开 `CMakeLists.txt` 把 `USE_RK_HW_DECODER` 设置为 `ON`。

### Nvidia Jetson

依赖 : `jetson_multimedia_api`,`libyuv` 。
打开 `CMakeLists.txt` 把 `USE_NV_HW_DECODER` 设置为 `ON`。

## 启动参数

以下是可用的启动参数

- `connection_delay`: 重新打开设备的延迟时间（以毫秒为单位）。某些设备，例如Astra mini，需要更长的时间来初始化，立即重新打开设备可能会在热插拔时导致固件崩溃。
- `enable_point_cloud`: 是否开启点云。
- `enable_colored_point_cloud`: 是否开启彩色点云。
- `point_cloud_qos`, `[color|depth|ir]_qos,``[color|depth|ir]_camera_info_qos`:
  ROS2消息质量服务（QoS）设置。可能的值是`SYSTEM_DEFAULT`，`DEFAULT`，`PARAMETER_EVENTS`，`SERVICES_DEFAULT`，`PARAMETERS`，`SENSOR_DATA`
  ，并且不区分大小写。这些对应于`rmw_qos_profile_system_default`，`rmw_qos_profile_default`，`rmw_qos_profile_parameter_events`，`rmw_qos_profile_services_default`，`rmw_qos_profile_parameters`。
- `enable_d2c_viewer`: 发布D2C叠加图像（仅用于测试）。
- `device_num`: 设备数量。如果需要多个相机，则必须填写此项。
- `color_width`, `color_height`, `color_fps`: 彩色流的分辨率和帧率。
- `ir_width`, `ir_height`, `ir_fps`: 红外流的分辨率和帧率。
- `depth_width`, `depth_height`, `depth_fps`: 深度流的分辨率和帧率。
- `enable_color`: 是否启用RGB相机。
- `enable_depth`: 启用深度相机。
- `enable_ir`: 是否开启IR相机。
- `depth_registration`: 是否启用将深度帧硬件对齐到彩色帧。当`enable_colored_point_cloud`设置为`true`
  时，此字段也要设置为`true`。
- `usb_port` : USB端口号，当使用多个相机时，这是必需的。
- `enable_accel`: 是否启用加速度计。
- `accel_rate` : 加速度计的频率， 可选值为`1.5625hz`,`3.125hz`,`6.25hz`,`12.5hz`,`25hz`,`50hz`,
  `100hz`,`200hz`,`500hz`,`1khz`,`2khz`,`4khz`,`8khz`,`16khz`,`32khz`。具体的值取决于当前的相机型号。
- `accel_range` : 加速度计的量程, 可选值为`2g`,`4g`,`8g`,`16g`。具体的值取决于当前的相机型号。
- `enable_gyro` : 是否启用陀螺仪。
- `gyro_rate` : 陀螺仪的频率, 可选值为`1.5625hz`,`3.125hz`,`6.25hz`,`12.5hz`,`25hz`,`50hz`,
  `100hz`,`200hz`,`500hz`,`1khz`,`2khz`,`4khz`,`8khz`,`16khz`,`32khz`。具体的值取决于当前的相机型号。
- `gyro_range` : 陀螺仪的量程, 可选值为`16dps`,`31dps`,`62dps`,`125dps`,`250dps`,`500dps`,`1000dps`,`2000dps`
  。具体的值取决于当前的相机型号。
- `enumerate_net_device` : 是否开启枚举网络设备的功能，true为开启，false为关闭，仅Femto mega和Gemini 2 XL设备支持。
  当通过网络方式访问以上设备时，需提前配置好设备的IP地址，使能开关需要配置成true。
- `depth_filter_config` : 配置深度滤波配置文件加载路径，默认深度滤波配置文件在/config/depthfilter目录下,仅gemini2支持。

## 深度模式切换：
- 启动相机前,可通过配置对应相继的xxx.launch.py的深度模式(depth_work_mode)支持。
- 深度模式切换支持的相机:Gemini 2, Gemini 2 L,Gemini 2 XL。
- xxx.launch.py默认深度模式配置为相机默认配置,如果需要修改可根据需要切换相应深度模式。
- 具体相机深度模式支持类型可查看深度模式的注释。

```python
    # Depth work mode support is as follows:
    # Unbinned Dense Default
    # Unbinned Sparse Default
    # Binned Sparse Default
    # Obstacle Avoidance
    DeclareLaunchArgument('depth_work_mode', default_value='')
```

- 深度工作模式查看:

```bash
ros2 run orbbec_camera list_depth_work_mode_node
```

## 深度NFOV和WFOV模式配置
对于Femto Mega和Femto Bolt设备，深度NFOV和WFOV模式通过在launch文件中配置Depth和IR的分辨率实现。
在launch文件中，`depth_width`, `depth_height`、'ir_width'、‘ir_height’为深度流的分辨率和IR流的分辨率。
IR的分辨率和帧率必须和深度保持一致。不同模式和分辨率对应关系如下:
- NFoV unbinned：640 x 576。
- NFoV binned：320 x 288。
- WFoV unbinned：1024 x1024。
- WFoV binned：512 x 512。

## 查看相机支持那些Profile

  ```bash
  ros2 run orbbec_camera list_camera_profile_mode_node
  ```

## 启动文件

| product serials                              | launch file             |
|----------------------------------------------|-------------------------|
| astra+                                       | astra_adv.launch.py     |
| astra mini /astra mini pro /astra pro        | astra.launch.py         |
| astra mini pro s                             | astra.launch.py         |
| astra2                                       | astra2.launch.py        |
| astra stereo s                               | stereo_s_u3.launch.py   |
| astra pro2                                   | astra_pro2.launch.py    |
| dabai                                        | dabai.launch.py         |
| dabai d1                                     | dabai_d1.launch.py      |
| dabai dcw                                    | dabai_dcw.launch.py     |
| dabai dw                                     | dabai_dw.launch.py      |
| dabai pro                                    | dabai_pro.launch.py     |
| deeya                                        | deeya.launch.py         |
| femto  /femto w                              | femto.launch.py         |
| femto mega                                   | femto_mega.launch.py    |
| femto bolt                                   | femto_bolt.launch.py    |
| gemini                                       | gemini.launch.py        |
| gemini                                       | gemini.launch.py        |
| gemini2 / dabai DCL                          | gemini2.launch.py       |
| gemini2L                                     | gemini2L.launch.py      |
| gemini e                                     | gemini_e.launch.py      |
| gemini e lite                                | gemini_e_lite.launch.py |
| dabai max                                    | dabai_max.launch.py     |
| dabai max pro                                | dabai_max_pro.launch.py |
| gemini uw                                    | gemini_uw.launch.py     |
| dabai dcw2                                   | dabai_dcw2.launch.py    |
| dabai dw2                                    | dabai_dw2.launch.py     |
| gemini ew                                    | gemini_ew.launch.py     |
| gemini ew lite                               | gemini_ew_lite.launch.py|

实际上所有的launch文件都是差不多的内容，只是相机的分辨率和图像格式不同。
XML launch 文件将会在后续的版本中删除。

## 已经支持的硬件产品列表

| **products list** | **firmware version**                      |
|-------------------|-------------------------------------------|
| Gemini 2 XL       | Obox: V1.2.5  VL:1.4.54                   |
| Astra 2           | 2.8.20                                    |
| Gemini 2 L        | 1.4.32                                    |
| Gemini 2          | 1.4.60 /1.4.76                            |
| Femto Mega        | 1.1.7  (window10、ubuntu20.04、ubuntu22.04) |
| Astra+            | 1.0.22/1.0.21/1.0.20/1.0.19               |
| Femto             | 1.6.7                                     |
| Femto W           | 1.1.8                                     |
| Femto Bolt        | 1.0.6 (unsupported ARM32)                 |
| DaBai             | 2436                                      |
| DaBai DCW         | 2460                                      |
| DaBai DW          | 2606                                      |
| Astra Mini Pro    | 1007                                      |
| Gemini E          | 3460                                      |
| Gemini E Lite     | 3606                                      |
| Gemini            | 3.0.18                                    |
| Astra Mini S Pro  | 1.0.05                                    |

## DDS Tuning

默认的DDS设置（humble）可能不适用于数据传输。不同的DDS设置可能具有不同的性能。在此示例中，我们使用CycloneDDS。有关更详细的信息，请参考[ROS DDS Tuning](https://docs.ros.org/en/humble/How-To-Guides/DDS-tuning.html)。

● 编辑CycloneDDS配置文件.

```bash
sudo gedit /etc/cyclonedds/config.xml
```

Add

```xml
<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
            xsi:schemaLocation="https://cdds.io/confighttps://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
    <Domain id="any">
        <General>
            <NetworkInterfaceAddress>lo</NetworkInterfaceAddress>
            <AllowMulticast>false</AllowMulticast>
        </General>
        <Internal>
            <MinimumSocketReceiveBufferSize>16MB</MinimumSocketReceiveBufferSize>
        </Internal>
        <Discovery>
            <ParticipantIndex>auto</ParticipantIndex>
            <MaxAutoParticipantIndex>30</MaxAutoParticipantIndex>
            <Peers>
                <Peer address="localhost"/>
            </Peers>
        </Discovery>
    </Domain>
</CycloneDDS>
```

● 设置环境变量，把下面变量添加到.zshrc或.bashrc文件中。

```bash
export ROS_DOMAIN_ID=42 # Numbers from 0 to 232
export ROS_LOCALHOST_ONLY=1
export CYCLONEDDS_URI=file:///etc/cyclonedds/config.xml
```

提示：要了解为什么最大的ROS_DOMAIN_ID是232
请参考[The ROS DOMAIN ID](https://docs.ros.org/en/humble/Concepts/About-Domain-ID.html)
● 增加UDP接收缓冲区大小。编辑

```bash
/etc/sysctl.d/10-cyclone-max.conf
```

添加

```bash
net.core.rmem_max=2147483647
net.core.rmem_default=2147483647
```

## 常见问题

多相机没有图像

- 可能是USB供电不足导致，可以尝试用供电的USB hub。

- 也可能是分辨率设置太高，降低分辨率后再尝试。

为什么这里这么多launch文件

- 存在多个launch文件的原因是因为不同相机的默认分辨率和图像格式不同。为了更方便使用，已将启动文件分开为每个相机一个。

## License

Copyright 2023 Orbbec Ltd.

Licensed under the Apache License, Version 2.0 (the "License"); you may not use this project except in compliance with
the License. You may obtain a copy of the License at

[http://www.apache.org/licenses/LICENSE-2.0](http://www.apache.org/licenses/LICENSE-2.0)

Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "
AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific
language governing permissions and limitations under the License.

**Other names and brands may be claimed as the property of others**
