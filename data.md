
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

ros2 interface show lightning/msg/NavState
ros2 topic echo /lightning/nav_state
```

#### 保存地图
```bash
source /opt/ros/foxy/setup.bash
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


### 实机录制 bag
```bash
sudo su
source /opt/robot/scripts/setup_ros2.sh
到106主机开通服务 systemctl start multicast-relay.service
```
在104主机检查 
ping 10.21.31.106
systemctl list-units --type=service --state=running

taskset -c 4,5,6,7 chrt 90 ros2 bag record -o lio260310 /tf /IMU /LIDAR/POINTS 

scp user@10.21.31.104:/home/user/lightning_ws/lio260305.tar.gz /mnt/e/data/


### 数据内容

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

### 运行结果
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
