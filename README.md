# Lightning-LM Deployment Guide for Deep Robotics M20

This guide outlines the specific steps, configurations, and commands required to deploy Lightning-LM on the Deep Robotics M20 platform equipped with a RoboSense LiDAR.

## 1. Dataset & Hardware Setup

Before deploying on the physical robot, it is highly recommended to obtain the code and test the algorithm using the provided dataset.

### Clone the Repository
Start by cloning the Deep Robotics specific version of the repository:
```bash
git clone https://github.com/DeepRoboticsLab/lightning-lm-deep-robotics.git
cd lightning-lm-deep-robotics
```

### M20 Dataset
Download the test dataset collected on the M20 robot here:
*   **Google Drive:** [M20 Robot Dataset](https://drive.google.com/drive/folders/19T__ai6u5WCTwyWi3L4KC9gVZI-jiQM-?usp=drive_link)
*   *Note: This dataset contains `lidar_data_bag`, which is used in the examples below.*

### M20 Hardware Configuration
For details on how to configure and use the RoboSense LiDAR specific to the M20 robot, refer to the official documentation:
*   **LiDAR Usage Guide:** [Deep Robotics M20 LiDAR Docs](https://alidocs.dingtalk.com/i/p/OlnXRl7ed542DGLp/docs/qnYMoO1rWxDL7r6LHbad2kA9W47Z3je9)

## 2. Build Instructions

### Step 1: Install APT Dependencies

Do **not** install `libgoogle-glog-dev` from apt — it conflicts with the thirdparty glog v0.6.0 (both register the same gflags flags at startup, causing a crash on launch). Install everything else:

```bash
sudo apt install libopencv-dev libpcl-dev pcl-tools libyaml-cpp-dev libgflags-dev ros-humble-pcl-conversions
```

If `libgoogle-glog-dev` is already installed, remove it:
```bash
sudo apt remove libgoogle-glog-dev libgoogle-glog0v5
```

### Step 2: Build glog v0.6.0 from Thirdparty

```bash
cd src/lightning-lm-deep-robotics/thirdparty/glog
mkdir build && cd build
cmake -DBUILD_SHARED_LIBS=ON -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)
sudo make install
sudo ldconfig
cd ../../../../..
```

### Step 3: Build Pangolin v0.9.3 from Thirdparty

```bash
cd src/lightning-lm-deep-robotics/thirdparty/Pangolin
mkdir build && cd build
cmake -DBUILD_EXAMPLES=OFF -DBUILD_TOOLS=OFF -DCMAKE_CXX_FLAGS="-Wno-error" -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)
sudo make install
sudo ldconfig
cd ../../../../..
```

### Step 4: Build lightning-lm

**Standard build** (PC/Server with sufficient RAM):
```bash
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

**Low memory build** (recommended for on-board computers such as M20/RK3588, to avoid OOM crashes):
```bash
export MAKEFLAGS="-j4"
source /opt/ros/humble/setup.bash
colcon build --parallel-workers 4 --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```
Compilation on RK3588 takes approximately 20 minutes.

## 3. Configuration

The primary configuration file for the M20 robot is located at:
`src/lightning-lm-deep-robotics/config/default_deep_robotics.yaml`

**Key Configuration Parameters:**
*   **LiDAR Type:** Ensure `fasterlio.lidar_type` is set to `4` (RoboSense).
*   **Topics:** Check that `common.lidar_topic` and `common.imu_topic` match the sensor output in your bag or live stream.

## 4. Mapping (SLAM)

### Option A: Real-time Mapping (Online)
*Suitable for testing on the robot or playing back bags in real-time simulation.*

1.  **Play the ROS2 bag:**
    ```bash
    ros2 bag play ~/Downloads/m20/lidar_data_bag --clock
    ```
    *Note: Real-time processing is resource-intensive. On WSL/Virtual Machines, playback speed might need to be reduced.*

2.  **Launch the Online SLAM Node:**
    ```bash
    ros2 run lightning run_slam_online --config src/lightning-lm-deep-robotics/config/default_deep_robotics.yaml
    ```

3.  **Save the Map:**
    Once mapping is complete, save the result to disk:
    ```bash
    ros2 service call lightning/save_map lightning/srv/SaveMap "{map_id: new_map}"
    ```
4. **Log the localization state:**
    Check the realtime odometry of SLAM by running `ros2 topic echo /lightning/nav_state`, it will list position, attitude quaternion and velocity.


### Option B: Offline Mapping (Fast)
*Recommended for quickly generating maps from recorded data without dropping frames.*

1.  **Run Offline SLAM:**
    ```bash
    ros2 run lightning run_slam_offline --input_bag /home/msy/Downloads/m20/lidar_data_bag/lidar_data_bag_0.db3 --config ./src/lightning-lm-deep-robotics/config/default_deep_robotics.yaml
    ```
    *Note: The system automatically saves results to the `data/new_map` directory upon completion.*

### Viewing the Map Results
*   **3D Point Cloud:**
    ```bash
    pcl_viewer ./data/new_map/global.pcd
    ```
*   **2D Grid Map:**
    ```bash
    sudo apt install feh
    feh data/new_map/map.pgm
    ```

## 5. Localization

### Option A: Real-time Localization (Online)
*No UI is shown by default for this mode.*

1.  **Play the ROS2 bag (or run on live robot):**
    ```bash
    ros2 bag play ~/Downloads/m20/lidar_data_bag --clock
    ```

2.  **Launch the Localization Node:**
    *   Ensure `system.map_path` in your yaml config points to the folder containing the map (default: `new_map`).
    *   **Run command:**
        ```bash
        ros2 run lightning run_loc_online --config ./src/lightning-lm-deep-robotics/config/default_deep_roboticsloc.yaml
        ```

### Option B: Offline Localization
Run localization on a bag file without real-time constraints to verify algorithm performance.
```bash
ros2 run lightning run_loc_offline --config ./src/lightning-lm-deep-robotics/config/default_deep_roboticsloc.yaml --input_bag [path_to_bag]
```

## 6. M20 Hardware Deployment
We test on the AOS platform.

### 6.1 Hardware Configuration
#### 6.1.1 Networking
Connect the AOS host to the network.
Modify `vim /etc/NetworkManager/NetworkManager.conf` and delete `unmanaged-devices` and all the `[keyfile]` section and reboot.

Running `nmcli d wifi list` should then display all available WiFi networks.
Set the name and connect using:
`sudo nmcli d wifi connect "<wifiname>" password "password" ifname wlan0`.

To maintain a continuous rviz display while the robot is moving, ensure a persistent WiFi connection between your computer and the M20 robot. is maintained.

#### 6.1.2 Point Cloud Permissions
Enable the service on the NOS host and select the `user` account.
```bash
ssh user@10.21.31.106
user@host.v1.4:~$ systemctl start multicast-relay.service
```
Once complete, switch to the target host, enter `su` mode, and check the point cloud:
```bash
source /opt/robot/scripts/setup_ros2.sh
ros2 topic hz /LIDAR/POINTS
```
These steps are required for each SLAM try, in other words, always check LiDAR topic accessibility first.

### 6.2 Preparation
#### 6.2.1 Dependencies

Follow **Steps 1–4** from the [Build Instructions](#2-build-instructions) section above. On M20, use the Low Memory Build in Step 4.

> **Note:** On M20 the ROS 2 distro is Foxy — replace `ros-humble-pcl-conversions` with `ros-foxy-pcl-conversions` in the apt install command, and source `/opt/ros/foxy/setup.bash` instead of humble.

### Visualization Issues
3D UI window in `run_slam_online` crash when starting the `pangolin`, primarily due to **OpenGL/EGL context initialization failure**. Error is `eglGetBindAPI(0x30a2) failed: EGL_BAD_PARAMETER (300c)`.
Due to compatibility issues with EGL + OpenGL support on the RK3588, Pangolin visualization (and other OpenGL applications) may fail to open.

Therefore, this modified version primarily utilizes **rviz2 for visualization**.

## 7. SLAM Deployment
Make sure:
- The robot is standing when starting program, since system may substract under ground points.
- Verify that the input point cloud topic is visible.
- If using rviz2 for visualization, check that `pub_tf` is enabled.
- Dynamic obstacles like cars and people may degrade system performance. In narrow tunnels or when the LiDAR is obstructed, localization may be lost.

### 7.1 start SLAM/Loc online
To facilitate debugging in the terminal, you can use tmux to open 3 windows by running `./start_lg_rviz_session.sh`. Switch between windows, check if all commands works, if not,  re-enter the corresponding commands as needed.

#### 7.1.1 Mapping mode
```bash
ros2 run lightning run_slam_online --config src/lightning-lm-deep-robotics/config/default_deep_roboticsslam.yaml
rviz2 -d src/lightning-lm-deep-robotics/config/showbodypc.rviz
ros2 topic echo /lightning/nav_state
ros2 service call /lightning/save_map lightning/srv/SaveMap "{map_id: 'office4'}"
```

#### 7.1.2 Localization mode
The procedure is basically the same as SLAM, but you need to ensure the map configuration is correct. Check:
```yaml
system:
  map_path: ./data/office4/
```
And verify the point cloud with `pcl_viewer ./data/office4/global.pcl`.

```bash
ros2 run lightning run_loc_online --config src/lightning-lm-deep-robotics/config/default_deep_roboticsloc.yaml
rviz2 -d src/lightning-lm-deep-robotics/config/showglobalmap.rviz
ros2 topic echo /lightning/nav_state
```

### 7.2 tf Check
We publish the `map->lidar_link` transform in `slamOnline` for visualization. Note that `pub_tf` only exists in the `LocSystem` of the original version.

```bash
ros2 run tf2_tools view_frames
ros2 topic echo /tf
ros2 run tf2_ros tf2_echo base_link lidar_link
ros2 run tf2_ros tf2_echo map base_link
```

#### 7.2.1 tmux session usage
Use `Ctrl+b` followed by `0`/`1`/`2` to switch between the 3 sub-windows.
```bash
Ctrl+b, d # Detach from session
Ctrl+b, c # Create a new tab
tmux attach -t lg # Re-attach to the session
tmux kill-session -t lg # Kill the session
```
If the session is disconnected due to network lag, you can attach to this session again. After MobaXterm reconnecting, in `su` mode run `tmux attach -t lg`.

### 7.3 Check the result
#### 7.3.1 log pose and velocity
Trun on `system.pub_odom` and `ros2 topic echo /lightning/nav_state` to list position, attitude quaternion and velocity.

Also you can run `ros2 service call /lightning/save_path lightning/srv/SavePath "{file_path: 'data/traj.txt'}" ` any time to save a TUM style trajectory.

#### 7.3.2 rviz2 for Real-time Visualization
Although the original Pangolin interface is more efficient, rviz2 visualization is also provided.

The rviz display :
- tf, `map->lidar_link`
- Odometry, `/lightning/odom`
- PointCloud2-currentScan, `/current_scan_cloud`
- PointCloud2-globalMap, `/global_map_cloud`
- Path: `/lightning/path`
The currentScan is transformed to the global 'map' frame and processed by Undistortion and Downsampling.  For SLAM mode, the globalMap is updated by KeyFrame updating. For Localization mode, the global map is set by the input file, which remains constant during online operation.

Note in original SLAM mode, default `pub_tf` is off.

```bash
rviz2 -d src/lightning-lm-deep-robotics/config/showbodypc.rviz
```
If `system.pub_tf: true`, the `/lightning/odom` odometry topic will be visible here.


When using localization mode, visualization:
```bash
rviz2 -d src/lightning-lm-deep-robotics/config/showglobalmap.rviz
```


### 7.4 Mapping mode configuration
The following options are added, with respect to the original version:
1. Print or publish localization results and odometry messages.
2. Optionally publish keyframe point clouds.
```yaml
system:
  log_pose_opt: false               # Print pos/vel directly to terminal
  pub_odom: true                    # Publish odometry topic
  enable_lidar_loc_rviz: false      # Enable RViz point cloud publishing
  rviz_current_scan_topic: "/current_scan_cloud"
  rviz_global_map_topic: "/global_map_cloud"
  enable_path_rviz: true            
  pub_tf: true                      
```

### 7.5 Localization mode configuration
Firstly check the map_path is correct. If you need to manually set another initial pose, use the following in the config:
```yaml
system:
  map_path: ./data/new_map/
  use_init_pose: true
  init_pos: [0.0, 0.0, 0.0]         # Initial position in the point cloud [x, y, z]
  init_quat: [0.0, 0.0, 0.0, 1.0]   # Initial quaternion relative to the global map [x, y, z, w]
```

### 7.6 Recodring bags

taskset -c 4,5,6,7 chrt 90 ros2 bag record -o lio260310 /tf /IMU /LIDAR/POINTS