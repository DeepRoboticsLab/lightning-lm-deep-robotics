# Lightning-LM Deployment Guide for Deep Robotics M20

This guide describes how to deploy Lightning-LM on the Deep Robotics M20 platform running ROS 2 Foxy and equipped with a RoboSense LiDAR. It covers the complete deployment procedure, configuration, and commonly used commands.

Tutorial videos:

* [YouTube](https://youtu.be/1S8X03tm3-8?si=aPQY8Id6GBd-21Uc)
* [Bilibili](https://www.bilibili.com/video/BV12YQZBqE1b/?share_source=copy_web&vd_source=57f46145c37bfb96f7583c9e02081590)

## Update Notes

Because the M20 has limited onboard computing power and memory, large-scale and high-density point clouds under the original configuration can be difficult to process in real time, resulting in computational delays and data backlog.

In this version, the LiDAR point-cloud processing range has been appropriately reduced to decrease the number of points involved in each frame. This reduces CPU and memory usage and improves the real-time performance and stability of Lightning-LM on the M20 platform.

## 1. Dataset and Hardware Preparation

Before deploying Lightning-LM on the physical robot, it is strongly recommended to obtain the source code and first test the algorithm using the provided datasets.

### Clone the Repository

Clone the Deep Robotics-specific version of Lightning-LM:

```bash
git clone https://github.com/DeepRoboticsLab/lightning-lm-deep-robotics.git
```

### M20 Dataset

Download the test dataset collected using the M20 robot:

* **Google Drive:** [M20 Robot Dataset](https://drive.google.com/drive/folders/19T__ai6u5WCTwyWi3L4KC9gVZI-jiQM-?usp=drive_link)

* **Note:** The dataset contains `lidar_data_bag`, which is used in the examples below.

A larger and more challenging dataset is also provided. This dataset was recorded while the M20 robot was moving near the Main Library of Zhejiang University:

[Dataset Download](https://drive.google.com/drive/folders/1dAoFarl1nb6sMvoKAEhMDwgeG8ujeo_f?usp=drive_link)

The Lite3 LiDAR dataset can be downloaded here:

[Lite3 LiDAR Dataset](https://drive.google.com/drive/folders/12alT4TuZwYC_xWUrex2quWOoMKXzpbQ2?usp=drive_link)

The following configuration file can be used to test the algorithm:

```text
lightning-lm-deep-robotics/config/default_livox.yaml
```

The dataset also contains videos recorded while the robot was collecting LiDAR data.

### M20 Hardware Configuration

For configuration and usage instructions for the RoboSense LiDAR installed on the M20 robot, refer to the official documentation:

* **LiDAR User Guide:** [Deep Robotics M20 LiDAR Docs](https://alidocs.dingtalk.com/i/p/OlnXRl7ed542DGLp/docs/qnYMoO1rWxDL7r6LHbad2kA9W47Z3je9)

## 2. Build Instructions

### Step 1: Install APT Dependencies

```bash
sudo apt install -y libopencv-dev libpcl-dev pcl-tools libyaml-cpp-dev libepoxy-dev libgflags-dev libgoogle-glog-dev python3-wheel ros-foxy-pcl-conversions
```

### Step 2: Extract and Build Pangolin 0.9.3

Enter the `thirdparty` directory and extract Pangolin:

```bash
cd ~/lightning-lm/thirdparty

unzip Pangolin-0.9.3.zip

cd Pangolin-0.9.3
```

Install the required dependencies:

```bash
sudo apt update

sudo apt install -y \
  cmake \
  g++ \
  pkg-config \
  libgl1-mesa-dev \
  libegl1-mesa-dev \
  libglew-dev \
  libepoxy-dev \
  libeigen3-dev \
  libx11-dev \
  libwayland-dev \
  libxkbcommon-dev \
  wayland-protocols
```

Create the build directory and configure the project using CMake:

```bash
mkdir -p build

cmake -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_EXAMPLES=OFF \
  -DBUILD_TOOLS=OFF \
  -DBUILD_PANGOLIN_PYTHON=OFF \
  -DBUILD_PANGOLIN_LIBOPENEXR=OFF
```

Compile Pangolin:

```bash
cmake --build build -j3
```

Install Pangolin after compilation:

```bash
sudo cmake --install build
```

Finally, refresh the dynamic library cache:

```bash
sudo ldconfig
```

If no errors occur during compilation, Pangolin 0.9.3 has been successfully built and installed.

### Step 3: Build Lightning-LM

**Low-memory build method**, recommended for onboard computers such as the M20 / RK3588. This can help prevent compilation failures caused by out-of-memory (OOM) errors:

```bash
cd lightning-lm-deep-robotics

export MAKEFLAGS="-j3"

source /opt/robot/scripts/setup_ros2.sh

colcon build --parallel-workers 3 --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/setup.bash
```

A complete build takes approximately 30 minutes on the RK3588.

Using four CPU cores for compilation may cause the system to freeze because of insufficient memory and OOM issues.

**Standard build method** for PCs or servers with sufficient memory:

```bash
cd lightning-lm-deep-robotics

source /opt/ros/foxy/setup.bash

colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/setup.bash
```

## 3. Configuration

The main configuration file for the M20 robot is:

```text
config/default_deep_roboticsslam.yaml
```

**Key configuration parameters:**

* **LiDAR type:** Make sure `fasterlio.lidar_type` is set to `4`, which represents the RoboSense LiDAR.

* **Topics:** Check whether `common.lidar_topic` and `common.imu_topic` match the actual topics published by the rosbag or the physical robot sensors.

## 4. Mapping (SLAM)

### Option A: Online Mapping

This mode is suitable for physical robot testing or for simulating real-time operation by playing a rosbag at its original speed.

1. **Play the ROS 2 bag:**

```bash
ros2 bag play ~/Downloads/m20/lidar_data_bag --clock
```

**Note:**

* Replace `~/Downloads/m20/lidar_data_bag` with the actual path to your rosbag.

* Real-time processing requires significant computational resources. When running inside WSL or a virtual machine, it may be necessary to reduce the rosbag playback speed.

```bash
ros2 bag play ~/Downloads/m20/lidar_data_bag --clock -r 0.5
```

2. **Start the online SLAM node:**

```bash
ros2 run lightning run_slam_online --config config/default_deep_robotics.yaml
```

3. **Save the map:**

After mapping is complete, execute the following command. The map will be saved under the `/lightning-lm-deep-robotics/data` directory:

```bash
ros2 service call lightning/save_map lightning/srv/SaveMap "{map_id: new_map}"
```

4. **Check localization status:**

Use the following command to inspect the real-time SLAM odometry state:

```bash
ros2 topic echo /lightning/nav_state
```

The output includes:

* Position
* Orientation quaternion
* Velocity

### Option B: Offline Mapping

Offline mapping is recommended for quickly generating a map from previously recorded data while avoiding frame loss caused by real-time processing limitations.

1. **Run offline SLAM:**

```bash
ros2 run lightning run_slam_offline --input_bag path/to/lidar_data_bag_0.db3 --config config/default_deep_robotics.yaml
```

**Note:** After the program finishes, the results are automatically saved to:

```text
data/new_map
```

### View Mapping Results

**3D point-cloud map on the server:**

```bash
pcl_viewer ./data/new_map/global.pcd
```

**3D point-cloud map over SSH:**

It is recommended to transfer the `data` directory to the local computer and open the map using CloudCompare.

**2D occupancy grid map:**

```bash
sudo apt install feh

feh data/new_map/map.pgm
```

## 5. Localization

### Option A: Online Localization

This mode does not display a UI by default.

1. **Play a ROS 2 bag or directly use the physical robot sensors:**

```bash
ros2 bag play ~/Downloads/m20/lidar_data_bag --clock
```

2. **Start the localization node:**

Make sure that:

```text
system.map_path
```

in the YAML configuration points to the directory containing the map files. The default map is `new_map`.

Run:

```bash
ros2 run lightning run_loc_online --config config/default_deep_roboticsloc.yaml
```

### Option B: Offline Localization

Offline localization can process a rosbag without real-time constraints and is useful for validating algorithm performance:

```bash
ros2 run lightning run_loc_offline --config config/default_deep_roboticsloc.yaml --input_bag [path_to_bag]
```

## 6. M20 Deployment

The system was tested on the AOS (103) platform, which already has ROS 2 Foxy installed.

### 6.1 Hardware Configuration

#### 6.1.1 Network Configuration

Connect the AOS (103) host to the network.

Edit:

```bash
vim /etc/NetworkManager/NetworkManager.conf
```

Remove:

```text
unmanaged-devices
```

and the entire:

```text
[keyfile]
```

section, and then reboot the robot.

After rebooting, run:

```bash
nmcli d wifi list
```

The available Wi-Fi networks should now be displayed.

Connect to Wi-Fi using:

```bash
sudo nmcli d wifi connect "<wifiname>" password "password" ifname wlan0
```

To maintain continuous RViz visualization while the robot is moving, ensure that the computer and M20 maintain a stable and persistent Wi-Fi connection.

#### 6.1.2 Point-Cloud Access Permissions

Start the corresponding service on the NOS (106) host using the `user` account:

```bash
ssh user@10.21.31.106

sudo systemctl start multicast-relay.service
```

Check the service status:

```bash
sudo systemctl status multicast-relay.service
```

The service can also be enabled to start automatically after every robot reboot:

```bash
sudo systemctl enable multicast-relay.service
```

After this is complete, switch back to AOS (103) and enter `su` mode.

The password is `'`, i.e., a single English quotation mark.

Then check the LiDAR point cloud:

```bash
source /opt/robot/scripts/setup_ros2.sh

ros2 topic hz /LIDAR/POINTS
```

This check should be performed before every SLAM run.

In other words, **before each SLAM test, first confirm that the LiDAR point-cloud topic is accessible and publishing normally.**

### 6.2 Preparation

#### 6.2.1 Dependencies

Install the required dependencies and complete the compilation process according to the **Build Instructions** above.

#### 6.2.2 M20 Visualization Issue

The 3D UI window in `run_slam_online` may crash when Pangolin is initialized.

The main cause is:

**OpenGL / EGL context initialization failure.**

A typical error is:

```text
eglGetBindAPI(0x30a2) failed: EGL_BAD_PARAMETER (300c)
```

Due to compatibility issues with EGL + OpenGL on the RK3588 platform, Pangolin visualization and other OpenGL-based applications may fail to start correctly.

Therefore, this modified version primarily uses:

**RViz2**

for visualization.

### 6.3 Recording a rosbag

The following command can be used to record LIO-related real-time topics into a rosbag:

```bash
taskset -c 4,5,6,7 chrt 90 ros2 bag record -o lio260310 /tf /IMU /LIDAR/POINTS
```

## 7. SLAM Test

Before running SLAM, make sure that:

* The robot is standing when the program starts, because the system may remove point-cloud data below the estimated ground level.

* The input point-cloud topic is available.

* Dynamic obstacles such as vehicles and pedestrians may reduce system performance.

* Localization may be lost in narrow corridors or when the LiDAR is heavily occluded.

The following node is used for testing:

```text
run_slam_online
```

### 7.1 Manual Startup

Mapping mode requires at least four terminal windows.

Run:

```bash
ros2 run lightning run_slam_online --config config/default_deep_roboticsslam.yaml
```

Save the map:

```bash
ros2 service call /lightning/save_map lightning/srv/SaveMap "{map_id: 'new_map'}"
```

View the map:

```bash
pcl_viewer ./data/new_map/global.pcd
```

## 8. Localization Test

Localization testing uses the:

```text
run_loc_online
```

node.

The overall procedure is similar to SLAM, but the map configuration must be correct.

Check whether `map_path` in the loaded YAML configuration points to the correct point-cloud map directory.

For example:

```yaml
system:
  map_path: ./data/office30/global.pcd
```

Make sure that the map file is valid.

### 8.1 Start Online Localization

Localization mode requires at least three terminal windows.

Start the localization node:

```bash
ros2 run lightning run_loc_online --config config/default_deep_roboticsloc.yaml
```

Publish the PCD map:

```bash
ros2 run lightning pcd_map_publisher \
    --ros-args \
    -p pcd_path:=data/office30/global.pcd \
    -p voxel_size:=0.3
```

Start RViz2:

```bash
rviz2 -d config/default.rviz
```

## 9. Configuration and Result Verification

This project enables:

```text
pub_tf
```

by default for RViz2 visualization.

### 9.1 Mapping Mode Configuration

The following configuration options have been added to support:

1. Printing or publishing localization state and odometry messages.

2. Optionally publishing point clouds, trajectories, and other mapping results.

Add the following settings to the YAML configuration:

```yaml
system:
  log_pose_opt: false                   # Whether to print position/velocity directly in the terminal
  pub_odom: true                        # Whether to publish the odometry topic
  enable_lidar_loc_rviz: false          # Whether to enable RViz point-cloud publishing
  rviz_current_scan_topic: "/current_scan_cloud"
  rviz_global_map_topic: "/global_map_cloud"
  enable_path_rviz: true
  pub_tf: true
```

By default, point clouds are not published.

The default configuration provides basic functionality including:

* Trajectory saving
* `nav_state` state output

### 9.2 Localization Mode Configuration

When running `loc_online`, first check whether `map_path` is correct.

If another initial pose needs to be specified manually, use the following configuration:

```yaml
system:
  map_path: ./data/office30
  use_init_pose: true
  init_pos: [0.0, 0.0, 0.0]             # Initial position in the point-cloud map [x, y, z]
  init_quat: [0.0, 0.0, 0.0, 1.0]       # Initial orientation relative to the global map [x, y, z, w]
```

### 9.3 Using tmux Sessions

Press:

```text
Ctrl+b
```

and then press:

```text
0 / 1 / 2 / 4
```

to switch between the four sub-windows.

Common commands:

```bash
Ctrl+b, d # Detach from the current tmux session

Ctrl+b, c # Create a new window/tab

su

tmux attach -t lg # Reattach to the lg session

tmux kill-session -t lg # Terminate the lg session
```

If the SSH connection is interrupted because of a network failure, reconnect to the robot and reattach to the existing tmux session.

After reconnecting using MobaXterm, run the following command in `su` mode:

```bash
tmux attach -t lg
```

This returns to the programs that are still running inside the tmux session.

Use:

```text
Ctrl+b 0
```

to check the running status of the SLAM / Localization node.

### 9.4 RViz2 Real-Time Visualization

Although the original Pangolin interface is more efficient, this project also provides RViz2 visualization.

RViz displays:

* TF: `map -> lightning_base_link`
* Global PointCloud2 map: `/global_map`
* Path: `/lightning/path`

In SLAM mode:

```text
globalMap
```

is continuously updated as new keyframes are generated.

In Localization mode:

The global map is loaded from the input map file and remains unchanged during online localization.

Some of the topics listed above are not published every second.

**Note:**

RViz2 can be displayed normally through MobaXterm, but it cannot be displayed properly through a VSCode Remote window.

#### 9.4.1 Restarting RViz2 After a Network Reconnection

If the connection is interrupted because of a network failure, reconnect to the robot and restart RViz2 using:

```bash
source /opt/robot/scripts/setup_ros2.sh

pkill -f rviz2

rviz2 -d src/lightning-lm-deep-robotics/config/showbodypc.rviz
```
