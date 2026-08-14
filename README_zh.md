# Deep Robotics M20 上的 Lightning-LM 部署指南

本指南说明如何在搭载 RoboSense 激光雷达的 Deep Robotics M20 平台(foxy)上部署 Lightning-LM，包括具体步骤、配置方法以及相关命令。

教程视频：
- [YouTube](https://youtu.be/1S8X03tm3-8?si=aPQY8Id6GBd-21Uc)
- [Bilibili](https://www.bilibili.com/video/BV12YQZBqE1b/?share_source=copy_web&vd_source=57f46145c37bfb96f7583c9e02081590)

## 本次更新说明

由于 M20 板载算力和内存有限，原始配置下的大范围、高密度点云难以实时处理，容易造成计算延迟和数据积压。本次更新适当缩小了激光雷达点云的处理范围，减少每帧参与计算的点数，从而降低 CPU 和内存占用，提高 Lightning-LM 在 M20 实机上运行的实时性与稳定性。

## 1. 数据集与硬件准备

在部署到实体机器人之前，强烈建议先获取代码，并使用提供的数据集测试算法。

### 克隆仓库

首先，将 Deep Robotics 专用版本仓库克隆到你的工作空间源码目录：

```bash
git clone https://github.com/DeepRoboticsLab/lightning-lm-deep-robotics.git
```

### M20 数据集

下载在 M20 机器人上采集的测试数据集：

- **Google Drive：** [M20 Robot Dataset](https://drive.google.com/drive/folders/19T__ai6u5WCTwyWi3L4KC9gVZI-jiQM-?usp=drive_link)
- **注意：** 该数据集中包含 `lidar_data_bag`，下文示例会使用它。

我们还提供了一个更大、更困难的数据集，该数据集记录了 M20 机器人在浙江大学主图书馆附近运动时的数据：
[数据集下载](https://drive.google.com/drive/folders/1dAoFarl1nb6sMvoKAEhMDwgeG8ujeo_f?usp=drive_link)。

Lite3 激光雷达数据集可以从这里下载：
[Lite3 LiDAR Dataset](https://drive.google.com/drive/folders/12alT4TuZwYC_xWUrex2quWOoMKXzpbQ2?usp=drive_link)
可以使用：
`lightning-lm-deep-robotics/config/default_livox.yaml`
来测试算法。数据集中还包含机器人采集激光雷达数据时的实拍视频。

### M20 硬件配置

关于 M20 机器人所使用 RoboSense 激光雷达的配置和使用方法，请参考官方文档：

- **激光雷达使用指南：** [Deep Robotics M20 LiDAR Docs](https://alidocs.dingtalk.com/i/p/OlnXRl7ed542DGLp/docs/qnYMoO1rWxDL7r6LHbad2kA9W47Z3je9)

## 2. 编译说明

### 第 1 步：安装 APT 依赖

**不要**通过 apt 安装 `libgoogle-glog-dev`。

原因是它会与项目第三方目录中的 glog v0.6.0 冲突。两者在程序启动时会注册相同的 gflags 参数，从而导致程序启动崩溃。

安装其他依赖：

```bash
sudo apt install -y libopencv-dev libpcl-dev pcl-tools libyaml-cpp-dev libepoxy-dev libgflags-dev python3-wheel ros-humble-foxy-ersions
```

如果已经安装了 `libgoogle-glog-dev`，将其卸载：

```bash
sudo apt remove -y libgoogle-glog-dev libgoogle-glog0v5
```

### 第 2 步：编译 lightning-lm

**低内存编译方式**，推荐用于 M20 / RK3588 等机器人板载计算机，可避免 OOM 内存溢出导致编译失败：

```bash
cd lightning-lm-deep-robotics
export MAKEFLAGS="-j3"
source /opt/robot/scripts/setup_ros2.sh
colcon build --parallel-workers 3 --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

在 RK3588 上，完整编译大约需要 30 分钟。如果使用 4 个核心进行编译，系统可能因为内存不足（OOM）而卡死。

**标准编译方式**，（PC/服务器，内存充足）：

```bash
cd lightning-lm-deep-robotics
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## 3. 配置

M20 机器人的主要配置文件位于：

`lightning-lm-deep-robotics/config/default_deep_robotics.yaml`

**关键配置参数：**

- **激光雷达类型：** 确保 `fasterlio.lidar_type` 设置为 `4`，表示 RoboSense。
- **话题：** 检查 `common.lidar_topic` 和 `common.imu_topic` 是否与 rosbag 或实时机器人传感器实际输出的话题一致。

## 4. 建图（SLAM）

### 方案 A：实时建图（Online）

适用于机器人实机测试，或者按照实时速度播放 rosbag 进行模拟。

1. **播放 ROS2 bag：**

```bash
ros2 bag play ~/Downloads/m20/lidar_data_bag --clock
```

**注意：** 

- 请将~/Downloads/m20/lidar_data_bag改为实际路径。

- 实时处理对计算资源要求较高。如果在 WSL 或虚拟机中运行，可能需要降低 rosbag 播放速度。
```bash
ros2 bag play ~/Downloads/m20/lidar_data_bag --clock -r 0.5
```

2. **启动在线 SLAM 节点：**

```bash
ros2 run lightning run_slam_online --config config/default_deep_robotics.yaml
```

3. **保存地图：**

建图完成后，执行下列命令，地图将会保存在/lightning-lm-deep-robotics/data目录下：

```bash
ros2 service call lightning/save_map lightning/srv/SaveMap "{map_id: new_map}"
```

4. **查看定位状态：**

通过以下命令查看 SLAM 实时里程计状态：

```bash
ros2 topic echo /lightning/nav_state
```

其中会显示：

- 位置
- 姿态四元数
- 速度

### 方案 B：离线建图（Offline，速度更快）

推荐使用离线方式快速从已有数据中生成地图，同时避免实时运行过程中丢帧。

1. **运行离线 SLAM：**

```bash
ros2 run lightning run_slam_offline --input_bag path/to/lidar_data_bag_0.db3 --config config/default_deep_robotics.yaml
```

**注意：** 程序运行完成后会自动将结果保存到：
`data/new_map`
目录。

### 查看地图结果

**3D 点云地图：（服务器上）**

```bash
pcl_viewer ./data/new_map/global.pcd
```

**3D 点云地图：（ssh连接）**

推荐将data保存到本地使用CloudCompare打开


**2D 栅格地图：**

```bash
sudo apt install feh
feh data/new_map/map.pgm
```

## 5. 定位

### 方案 A：实时定位（Online）

该模式默认不会显示 UI。

1. **播放 ROS2 bag，或者直接运行实机传感器：**

```bash
ros2 bag play ~/Downloads/m20/lidar_data_bag --clock
```

2. **启动定位节点：**

确保 YAML 配置文件中的：

`system.map_path`

指向包含地图文件的目录，默认使用 `new_map`。

运行：

```bash
ros2 run lightning run_loc_online --config ./src/lightning-lm-deep-robotics/config/default_deep_roboticsloc.yaml
```

### 方案 B：离线定位

在不受实时运行限制的情况下，对 rosbag 进行定位，用于验证算法性能：

```bash
ros2 run lightning run_loc_offline --config ./src/lightning-lm-deep-robotics/config/default_deep_roboticsloc.yaml --input_bag [path_to_bag]
```

## 6. M20 实机部署

我们在 AOS（103）平台上进行了测试，该平台已经安装 ROS2 Foxy。

### 6.1 硬件配置

#### 6.1.1 网络配置

将 AOS（103）主机连接到网络。

修改：

```bash
vim /etc/NetworkManager/NetworkManager.conf
```

删除其中的：
`unmanaged-devices`
以及整个：
`[keyfile]`
部分，然后重启机器人。

之后运行：

```bash
nmcli d wifi list
```

能够显示所有可用 WiFi 网络。

通过以下命令连接 WiFi：

```bash
sudo nmcli d wifi connect "<wifiname>" password "password" ifname wlan0
```

为了让机器人运动过程中 RViz 能持续显示，需要保证电脑与 M20 机器人之间具有稳定、持续的 WiFi 连接。

#### 6.1.2 点云访问权限

在 NOS（106）主机上启动对应服务，并使用 `user` 用户。

```bash
ssh user@10.21.31.106
sudo systemctl start multicast-relay.service
```

查看服务状态：

```bash
sudo systemctl status multicast-relay.service
```

也可以直接设置为开机自动启动，这样以后每次机器人重启后都会自动运行：

```bash
sudo systemctl enable multicast-relay.service
```

完成后，切换回 AOS（103），进入 `su` 模式。

密码为：`'` 即一个英文单引号。

然后检查激光雷达点云：

```bash
source /opt/robot/scripts/setup_ros2.sh
ros2 topic hz /LIDAR/POINTS
```

每次运行 SLAM 前都需要进行这项检查。

也就是说，**每次 SLAM 测试之前都应首先确认 LiDAR 点云话题可以正常访问。**

### 6.2 准备工作

#### 6.2.1 依赖

按照前面 **Build Instructions** 中的步骤完成依赖安装和编译。

#### 6.2.2 M20 可视化问题

`run_slam_online` 中的 3D UI 窗口在启动 Pangolin 时可能崩溃。

主要原因是：

**OpenGL / EGL 上下文初始化失败。**

典型错误：

```text
eglGetBindAPI(0x30a2) failed: EGL_BAD_PARAMETER (300c)
```

由于 RK3588 平台上的 EGL + OpenGL 支持存在兼容性问题，Pangolin 可视化以及其他 OpenGL 应用可能无法正常打开。

因此，该修改版本主要使用：
**RViz2**
进行可视化。

### 6.3 录制 rosbag

可以使用以下命令，将与 LIO 相关的实时话题录制到 rosbag：

```bash
taskset -c 4,5,6,7 chrt 90 ros2 bag record -o lio260310 /tf /IMU /LIDAR/POINTS
```

## 7. SLAM 测试

运行 SLAM 前，需要确保：

- 启动程序时机器人应处于站立状态，因为系统可能会进行地面以下点云的剔除。
- 确认输入点云话题可以正常看到。
- 汽车、行人等动态障碍物可能降低系统性能。
- 在狭窄通道中，或者激光雷达被遮挡时，定位可能丢失。

因此这里使用：

`run_slam_online`

节点进行测试。

### 7.1 手动启动

建图模式至少需要 4 个窗口，分别运行以下内容：

```bash
ros2 run lightning run_slam_online --config src/lightning-lm-deep-robotics/config/default_deep_roboticsslam.yaml
```

```bash
rviz2 -d lightning-lm-deep-robotics/config/showbodypc.rviz
```

```bash
ros2 topic echo /lightning/nav_state
```

```bash
ros2 service call /lightning/save_map lightning/srv/SaveMap "{map_id: 'office4'}"
```

此时可以看到：

- `/lightning/odom`：里程计
- `/lightning/path`：`map` 坐标系下的轨迹

### 7.2 记录位姿和速度

在配置文件中开启：

```yaml
system.pub_odom
```

并运行：

```bash
ros2 topic echo /lightning/nav_state
```

即可输出：

- 位置
- 姿态四元数
- 速度

该信息不依赖 TF。

也可以随时调用以下服务保存轨迹：

```bash
ros2 service call /lightning/save_path lightning/srv/SavePath "{file_path: 'data/traj.txt'}"
```

保存的轨迹为 TUM 格式。

随后可以使用 Python 的 matplotlib 绘制轨迹：

```bash
python3 src/lightning-lm-deep-robotics/scripts/visualize_trajectory.py data/traj.txt
```

## 8. 定位测试

定位测试使用：

`run_loc_online`

节点。

整体流程与 SLAM 基本一致，但需要确保地图配置正确。

检查你所加载的 YAML 文件中的 `map_path` 是否指向正确的点云地图目录。

例如：

```yaml
system:
  map_path: ./data/office4/
```

确认地图文件是正常的


### 8.1 启动在线定位

定位模式至少需要 3 个窗口，分别运行：

```bash
ros2 run lightning run_loc_online --config lightning-lm-deep-robotics/config/default_deep_roboticsloc.yaml
```

```bash
rviz2 -d src/lightning-lm-deep-robotics/config/showglobalmap.rviz
```

```bash
ros2 topic echo /lightning/nav_state
```

## 9. 配置与结果检查

本项目默认开启：

`pub_tf`

用于 RViz2 可视化。

### 9.1 建图模式配置

为了实现：

1. 打印或发布定位状态以及里程计消息。
2. 可选地发布点云、轨迹等建图结果。

在 YAML 中添加了以下配置项：

```yaml
system:
  log_pose_opt: false               # 是否直接在终端打印位置/速度
  pub_odom: true                    # 是否发布里程计话题
  enable_lidar_loc_rviz: false      # 是否启用 RViz 点云发布
  rviz_current_scan_topic: "/current_scan_cloud"
  rviz_global_map_topic: "/global_map_cloud"
  enable_path_rviz: true
  pub_tf: true
```

默认情况下，点云不会发布。

配置文件默认提供：

- 轨迹保存
- `nav_state` 状态输出

等基本功能。

### 9.2 定位模式配置

运行 `loc_online` 时，首先检查：

`map_path`

是否正确。

如果需要手动指定另一组初始位姿，可以在配置文件中使用：

```yaml
system:
  map_path: ./data/office4/
  use_init_pose: true
  init_pos: [0.0, 0.0, 0.0]          # 点云地图中的初始位置 [x, y, z]
  init_quat: [0.0, 0.0, 0.0, 1.0]    # 相对于全局地图的初始四元数 [x, y, z, w]
```

### 9.3 TF 检查

为了可视化，我们在 `slamOnline` 中发布：

`map -> lidar_link`

变换。

需要注意，原版程序中 `pub_tf` 仅存在于 `LocSystem` 中。

可以使用以下命令检查 TF：

```bash
ros2 run tf2_tools view_frames
```

```bash
ros2 topic echo /tf
```

```bash
ros2 run tf2_ros tf2_echo base_link lidar_link
```

```bash
ros2 run tf2_ros tf2_echo map base_link
```

### 9.4 tmux 会话使用方法

使用：

```text
Ctrl+b
```

然后按：

```text
0 / 1 / 2 / 4
```

即可切换 4 个子窗口。

常用命令：

```bash
Ctrl+b, d # 从当前 tmux 会话中分离
Ctrl+b, c # 创建新的窗口/标签页
su
tmux attach -t lg # 重新进入 lg 会话
tmux kill-session -t lg # 删除 lg 会话
```

如果由于网络故障导致 SSH 连接中断，可以重新连接该 tmux 会话。

MobaXterm 重新连接后，在 `su` 模式下运行：

```bash
tmux attach -t lg
```

即可回到仍然运行中的程序。

使用：

```text
Ctrl+b 0
```

即可查看 SLAM / Localization 节点的运行状态。

### 9.5 RViz2 实时可视化说明

虽然原始 Pangolin 界面效率更高，但该项目也提供了 RViz2 可视化。

RViz 中显示：

- TF：`map -> lidar_link`
- Odometry：`/lightning/odom`
- PointCloud2 当前扫描：`/current_scan_cloud`
- PointCloud2 全局地图：`/global_map_cloud`
- Path：`/lightning/path`

`currentScan` 会被转换到全局 `map` 坐标系，并经过：

- 点云去畸变
- 降采样

处理。

在 SLAM 模式下：

`globalMap`

会随着关键帧更新而不断更新。

在 Localization 模式下：

全局地图由输入地图文件加载，在在线定位过程中保持不变。

上述部分话题并不是每秒都会发布。

**注意：**

RViz2 可以在 MobaXterm 中正常使用，但不能在 VSCode 远程窗口中正常显示。

#### 9.5.1 网络重连后重新启动 RViz2

如果因为网络故障导致连接中断，重新连接机器人后，可以使用以下方式重启 RViz2：

```bash
source /opt/robot/scripts/setup_ros2.sh
pkill -f rviz2
rviz2 -d src/lightning-lm-deep-robotics/config/showbodypc.rviz
```
