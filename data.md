
准备数据
4话题
rosbags-convert --src go4_turn99_2025-11-19-17-13-50.bag --dst go4/turn --dst-typestore ros2_humble     20Hz
rosbags-convert --src go4_2fast_2025-11-19-17-30-33.bag --dst go4/fast2                                 20Hz

rosbags-convert --src go_garage_2025-10-28-16-30-25.bag --dst go1 --dst-typestore ros2_humble           10Hz
rosbags-convert --src 2023-12-12-13-17-50.bag --dst plant --dst-typestore ros2_humble                   10Hz
1话题
以下这两个包都有tf： odom->base_link 
rosbags-convert   --src 2025-07-07-14-23-07.bag   --dst tunnel1   --dst-typestore ros2_humble           10Hz
rosbags-convert   --src 2025-07-07-14-53-11.bag   --dst tunnel2   --dst-typestore ros2_humble           10Hz
手动替换[]为"[]"，把offered_qos_profiles:都替换成 "[]"


colcon build --packages-select lightning --parallel-workers 2
free -h 检查剩余内存

<!-- 检查进度条卡在哪了 -->
colcon build --packages-select lightning --parallel-workers 1 --event-handlers console_direct+
<!-- 单线程编译包： 类似 make -j1  -->
CMAKE_BUILD_PARALLEL_LEVEL=1 colcon build --packages-select lightning --symlink-install --parallel-workers 1 --event-handlers console_direct+
确保系统有10G以上的内存，否则很有可能因内存不足而编译中断。正常编译时间为3min。

### 编译Pangolin

sudo apt install libgflags-dev
cmake -DBUILD_EXAMPLES=OFF -DBUILD_TOOLS=OFF -DCMAKE_CXX_FLAGS="-Wno-error" ..

date +%s
sudo date -s "2026-03-06 13:40:00"
sudo date -s "@1772969266"
source /opt/ros/foxy/setup.bash

CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --packages-select lightning --symlink-install
export MAKEFLAGS="-j1"
colcon build
编译好了如果没改，直接给 lightning-lm 创建一个 COLCON_IGNORE


scp user@10.21.31.103:/home/user/deb/libgflags-dev_2.2.2-1build1_arm64.deb .
scp libgflags-dev_2.2.2-1build1_arm64.deb user@10.21.31.104:/home/user/lightning_ws/third/libgflags-dev_2.2.2-1build1_arm64.deb

### 运行
vim src/lightning-lm/config/default_deep_robotics.yaml

```bash
ros2 topic hz /LIDAR/POINTS

ros2 run lightning run_slam_online --config ./src/lightning-lm/config/default_deep_roboticsslam.yaml
```
<!-- 
开启3D UI 会直接崩溃:
 ros2 run lightning run_slam_online --config ./src/lightning-lm/config/default.yaml
I20240724 21:11:59.484977  4450 laser_mapping.cc:14] init laser mapping from ./src/lightning-lm/config/default_deep_robotics.yaml
I20240724 21:11:59.491555  4450 laser_mapping.cc:74] lidar_type 4
I20240724 21:11:59.491621  4450 laser_mapping.cc:86] Using RoboSense Lidar
I20240724 21:11:59.495266  4450 slam.cc:39] slam with loop closing
I20240724 21:11:59.499159  4450 loop_closing.cc:55] loop closing module is running in online mode
I20240724 21:11:59.499384  4450 slam.cc:47] slam with 3D UI
error: eglBindAPI(0x30a2) failed: EGL_BAD_PARAMETER (300c)
 -->


#### 保存地图
```bash
source /opt/robot/scripts/setup_ros2.sh && source install/setup.bash

ros2 service call /lightning/save_map lightning/srv/SaveMap "{map_id: 'office4f'}"
```

ros2 run lightning run_slam_offline --input_bag /mnt/e/data/NCLT/20120115/20120115-002.db3 --config ./src/lightning-lm/config/default_nclt.yaml

ros2 run lightning run_slam_offline --input_bag /mnt/e/data/lidar_data_bag/lidar_data_bag_0.db3 --config ./src/lightning-lm/config/default_deep_robotics.yaml

#### tmux 3个窗口运行
tmux new -s lg
```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link lidar_link
rviz2 -d src/lightning-lm/config/showbodypc.rviz
 

# 检查tf：LocSystem才会起效pub_tf，slamMode无法使用
ros2 run tf2_tools view_frames
ros2 topic echo /tf
ros2 run tf2_ros tf2_monitor
ros2 run tf2_ros tf2_echo map lidar_link
ros2 run tf2_ros tf2_echo base_link lidar_link
```

```log
root@host.v1.4:/home/user/lightinglm_ws# ros2 run tf2_ros tf2_monitor
Gathering data on all frames for 10 seconds...



RESULTS: for all Frames

Frames:
Frame: base_link, published by <no authority available>, Average Delay: 5.09318e+07, Max Delay: 5.09318e+07

All Broadcasters:
Node: <no authority available> 11.5919 Hz, Average Delay: 5.09318e+07 Max Delay: 5.09318e+07
```


#### glog
```bash
sudo rm -rf /usr/local/include/glog
user@host.v1.4:~/glog/build$ sudo rm -f /usr/local/lib/libglog.*
user@host.v1.4:~/glog/build$ sudo rm -f /usr/local/lib/libglog.so*
user@host.v1.4:~/glog/build$ sudo rm -f /usr/local/lib/pkgconfig/glog.pc
然后
sudo ldconfig
ldd install/lightning/lib/lightning/run_slam_online | grep -E "glog|gflags"
        libglog.so.0 => /lib/aarch64-linux-gnu/libglog.so.0 (0x0000007faf10d000)
        libgflags.so.2.2 => /lib/aarch64-linux-gnu/libgflags.so.2.2 (0x0000007faeeed000)
        libglog.so.1 => not found
        libglog.so.1 => not found
所以需要重新编译：

按你说的
sudo apt install libgoogle-glog-dev libgflags-dev
Reading package lists... Done
Building dependency tree
Reading state information... Done
libgflags-dev is already the newest version (2.2.2-1build1).
libgoogle-glog-dev is already the newest version (0.4.0-1build1).
The following packages were automatically installed and are no longer required:
  libopts25 sntp
Use 'sudo apt autoremove' to remove them.
0 upgraded, 0 newly installed, 0 to remove and 0 not upgraded.
user@host.v1.4:~/lightinglm_ws$ sudo ldconfig
user@host.v1.4:~/lightinglm_ws$ rm -rf build/lightning install/lightning
user@host.v1.4:~/lightinglm_ws$ colcon build --packages-select lightning
Starting >>> lightning
--- stderr: lightning
CMake Error at cmake/packages.cmake:1 (find_package):
  By not providing "Findglog.cmake" in CMAKE_MODULE_PATH this project has
  asked CMake to find a package configuration file provided by "glog", but
  CMake did not find one.

  Could not find a package configuration file provided by "glog" with any of
  the following names:

    glogConfig.cmake
    glog-config.cmake

  Add the installation prefix of "glog" to CMAKE_PREFIX_PATH or set
  "glog_DIR" to a directory containing one of the above files.  If "glog"
  provides a separate development package or SDK, be sure it has been
  installed.
Call Stack (most recent call first):
  CMakeLists.txt:33 (include)

```

### bag
```bash
sudo su
source /opt/robot/scripts/setup_ros2.sh
到106主机开通服务 systemctl start multicast-relay.service
```
在104主机检查 
ping 10.21.31.106
systemctl list-units --type=service --state=running

taskset -c 4,5,6,7 chrt 90 ros2 bag record -o lio260305 /tf /IMU /LIDAR/POINTS 

scp user@10.21.31.104:/home/user/lightning_ws/lio260305.tar.gz /mnt/e/data/

:~/lightning_ws$ ros2 topic list
/ALIGNED_POINTS
/BATTERY_CHARGE_ENABLE
/BATTERY_DATA
/CANCEL_NAV
/CHARGE_CMD
/CHARGE_STATUS
/CPU_103
/CPU_104
/CPU_106
/EXCEPTION_NOTIFICATION
/FAULT_STATUS
/FIBOCOM/net_rtk/gngga
/FIBOCOM/net_rtk/heading
/FULL_CLOUD_MAP
/GAIT
/GLOBAL_PLANNER_STATUS
/GOAL
/GPS
/GRIDS_ID
/GRID_MAP
/HANDLE_STEER
/HEIGHT_IMAGE
/HEIGHT_MAP_STATUS
/HES_STATUS
/IMU
/IMU_DATA
/IMU_DATA_10HZ
/JOINTS_CMD
/JOINTS_DATA
/JOINTS_DATA_10HZ
/LED/STATUS
/LIDAR/POINTS
/LIDAR/STATUS
/LOCATION_STATUS
/LOCATION_STATUS/MATCHING_ERROR
/LOC_BODY_POINTS
/MOTION_INFO
/MOTION_STATE
/MOTION_STATUS
/NAV_CMD
/NAV_STATUS
/ODOM
/OOA_STATUS
/PLANNER_STATUS
/REAL_STEER
/STEER
/TERRAIN_CLASSIFIER_STATUS
/TRACK_PATH
/fibo_fusion_pose
/fibo_fusion_state
/initialpose
/parameter_events
/rosout
/tf

### 编译结果

```shell

~/Downloads/livox-mid360/cic-localization-0812
   打不开
~/Downloads/livox-mid360/cmu-scafie
   /imu/data
   /lidar/scan livox_ros_driver2
~/Downloads/corridor/corridor01
   /camera_1/image_raw
   /imu/data
   /velodyne_packets

~/Downloads/go4/fast2
~/Downloads/go4/go1
~/Downloads/go4/turn
   /imu/data
   /joint_states
   /leg_odom
   /lidar_points_192_168_2_202   /lidar_points_192_168_2_203   /lidar_points_192_168_2_204   /lidar_points_192_168_2_205
   /livox/imu_192_168_2_202   /livox/imu_192_168_2_203   /livox/imu_192_168_2_204   /livox/imu_192_168_2_205
~/Downloads/go4/fast2_merged
~/Downloads/go4/turn_merged
   /imu/data
   /joint_states
   /leg_odom
   /lidar_points
   /livox/imu_192_168_2_202   /livox/imu_192_168_2_203   /livox/imu_192_168_2_204   /livox/imu_192_168_2_205
ros2 bag play /mnt/d/Develop/degraded/tunnel1
   /imu/data /leg_odom /lidar_points /camera/accel/sample /camera/gyro/sample /camera/odom/sample /odom /tf /tf_static
   


/mnt/e/data/cave1/cave01
/mnt/e/data/corridor02/corridor02
/mnt/e/data/floor01/floor0
   /imu/data /velodyne_points /camera_1/image_raw

/mnt/e/data/geode/offroad2_converted
/mnt/e/data/geode/offroad4_
/mnt/e/data/geode/shield1_
/mnt/e/data/geode/tunnel1_
/mnt/e/data/geode/urbantunnel1_
   /imu/data /livox/imu /livox/lidar /left_camera/image/compressed /right_camera/image/compressed

/mnt/e/data/plant1
    /imu/data   /leg_odom   /livox/lidar  /rtk_pos livox_ros_driver2/CustomMsg
/mnt/e/data/tunnel1
/mnt/e/data/tunnel2
   /imu/data /leg_odom /lidar_points /camera/accel/sample /camera/gyro/sample /camera/odom/sample /odom /tf /tf_static
```

ros2 run lightning run_slam_offline --input_bag ~/Downloads/go4/turn_merged --config ./lightning-lm2/config/mid360_116.yaml

>>> ===== Printing run time =====
I1120 14:36:16.838228 timer.cc:21] > [    IVox Add Points] average time usage: 0.0124458 ms, med: 0.009206 95%: 0.022307, called times : 995
I1120 14:36:16.838284 timer.cc:21] > [    Incremental Mapping] average time usage: 0.055171 ms, med: 0.051198 95%: 0.081537, called times : 995
I1120 14:36:16.838591 timer.cc:21] > [    ObsModel (IEKF Build Jacobian)] average time usage: 0.054769 ms, med: 0.026124 95%: 0.042536, called times : 2000
I1120 14:36:16.838699 timer.cc:21] > [    ObsModel (Lidar Match)] average time usage: 0.533155 ms, med: 0.446568 95%: 0.874594, called times : 2000
I1120 14:36:16.838717 timer.cc:21] > [G2P5 Occupancy Mapping] average time usage: 0.876652 ms, med: 0.810436 95%: 1.16552, called times : 41
I1120 14:36:16.839102 timer.cc:21] > [IEKF Solve and Update] average time usage: 2.2728 ms, med: 1.90675 95%: 3.75131, called times : 995
I1120 14:36:16.839171 timer.cc:21] > [Preprocess (Standard)] average time usage: 0.606729 ms, med: 0.421565 95%: 1.02764, called times : 1000
I1120 14:36:16.839224 timer.cc:21] > [Undistort Pcl] average time usage: 0.527184 ms, med: 0.495865 95%: 0.705673, called times : 996
I1120 14:36:16.839238 timer.cc:26] >>> ===== Printing run time end =====



ros2 run lightning run_slam_online --config ./config/mid360_116.yaml
```