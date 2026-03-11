## changelog
主要增加了以下几个选项用于
1. 打印或发布定位结果、里程计消息，
2. 可选发布关键帧点云
```yaml
system:
  log_pose_opt: false               # 命令行打印位置和速度
  pub_odom: true                    # 发布里程计话题
  enable_lidar_loc_rviz: false      # 是否启用RViz点云发布
  rviz_current_scan_topic: "/current_scan_cloud"
  rviz_global_map_topic: "/global_map_cloud"
  pub_tf: true                      # 原版LocSystem才会起效pub_tf，增加了 slamOnline 发布 map->lidar_link 的tf
```
定位结果在以下2个话题，如果pub_tf开启，则发布`/lightning/odom`
否则发布`/lightning/nav_state`，后者按localization_result定义，格式为`msg/NavState.msg`，它不需要pub_tf开启。
另外，log_pose_opt: true可直接在node终端位姿、速度。

发布点云需要tf开启，2个话题代表：1是当前关键帧到KF中的更新点、2是每KF更新后的globalMap点云。两者都经过了变换、配准、去畸变、降采样，程序中点云可能经过 motion compension, transform, distance filder, intensity filter, voxelize, ROI, ground substraction, ICP... 此处发布的是 laser_mapping/scan_down_world_ 的点云，经过了初步的处理。

建议不要在rviz2中可视化点云。通常会带来性能损失，src/ui/pangolin_window_impl.cc可视化当前帧点云更高效.

loc模式同样发布话题，调试rviz可行

### build
lightning-lm编译时间在RK3588上通常20min。
```bash
export MAKEFLAGS="-j2"
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --packages-select lightning --symlink-install
```

### 编译选项差异
已增加编译选项可自动适应RK3588或x86主机配置。

- 由于`src/ui/pangolin_window_impl.cc Line 56: CloudPtr tmp_cloud = std::make_shared<PointCloudType>(*(keyframe->GetCloud()));`的报错，增加选项使其在arm64上更改为`boost::make_shared`
- cmake/packages.cmake: 仅在 x86/x86_64 架构上启用 SSE 指令集

此外还添加了一些略微的相较lightning-lm的改动。

## usage
### SLAM Online 模式
首先确保sudo权限并能收到点云话题：
```bash
ros2 topic hz /LIDAR/POINTS
```

开启3个窗口，分别运行run_slam_online、rviz2、topic打印。首先分别：
```bash
root@host.v1.4:/home/user/lightinglm_ws# source /opt/robot/scripts/setup_ros2.sh && source install/setup.bash
```
然后SLAM模式：
```bash
ros2 run lightning run_slam_online --config src/lightning-lm/config/default_deep_roboticsslam.yaml
rviz2 -d src/lightning-lm/config/showbodypc.rviz
ros2 topic echo /lightning/nav_state
```
其中rviz中显示了 tf, Odometry, PointCloud2-currentScan, PointCloud2-globalMap.
如果话题不能正常打印，检查`ros2 interface show lightning/msg/NavState`

另外，获取当前建图的完整结果，可调用Service
```bash
ros2 service call /lightning/save_map lightning/srv/SaveMap "{map_id: 'office4'}"
```
### Loc 模式
流程与SLAM基本一样，但需要保证地图配置正确，检查
```yaml
system:
  map_path: ./data/office4/
```
并检查点云 `pcl_viewer ./data/office4/0.pcl`

Loc模式：
```bash
ros2 run lightning run_loc_online --config src/lightning-lm/config/default_deep_roboticsloc.yaml
rviz2 -d src/lightning-lm/config/showglobalmap.rviz
ros2 topic echo /lightning/nav_state
```
如果需要手动设置初值，可用config中的：
```yaml
system:
  map_path: ./data/office4/
  use_init_pose: true
  init_pos: [0.0, 0.0, 0.0] # 初始位置 [x, y, z]
  init_quat: [0.0, 0.0, 0.0, 1.0] # 初始四元数 [x, y, z, w]
```

### tmux 终端多窗口
为方便在终端上调试，用tmux开启3个窗口可直接运行`./lg_rviz_session.sh`，然后用Ctrl+b 0/1/2在3个子窗口间切换。如某个执行不正确，可手动切换或重新输入命令。
```bash
Ctrl+b, d # 退出
Ctrl+b, c # 创建新标签
tmux attach -t lg # 重新进入
tmux kill-session -t lg # 删除会话
```
