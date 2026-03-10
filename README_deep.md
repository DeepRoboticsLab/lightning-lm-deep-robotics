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

### Standard Build
For standard environments (PC/Server) with sufficient RAM:
```bash
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### Low Memory Build (Recommended for On-Board Computers)
If you are compiling directly on the M20 robot or a device with limited RAM (to avoid system freezes or crashes), use the following single-threaded build method:

```bash
export MAKEFLAGS="-j1"
source /opt/ros/humble/setup.bash
colcon build --parallel-workers 1 --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```
The compalitation could last about 20 minutes.

## 3. Configuration

The primary configuration file for the M20 robot is located at:
`src/lightning-lm/config/default_deep_robotics.yaml`

**Key Configuration Parameters:**
*   **LiDAR Type:** Ensure `fasterlio.lidar_type` is set to `4` (RoboSense).
*   **Topics:** Check that `common.lidar_topic` and `common.imu_topic` match the sensor output in your bag or live stream.

### Mapping mode configuration
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
  pub_tf: true                      # pub_tf only works in original LocSystem; added map->lidar_link tf publishing to slamOnline
```

### Localization mode configuration
Firstly check the map_path is correct. If you need to manually set another initial pose, use the following in the config:
```yaml
system:
  map_path: ./data/new_map/
  use_init_pose: true
  init_pos: [0.0, 0.0, 0.0]         # Initial position in the point cloud [x, y, z]
  init_quat: [0.0, 0.0, 0.0, 1.0]   # Initial quaternion relative to the global map [x, y, z, w]
```

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
    ros2 run lightning run_slam_online --config src/lightning-lm/config/default_deep_robotics.yaml
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
    ros2 run lightning run_slam_offline --input_bag /home/msy/Downloads/m20/lidar_data_bag/lidar_data_bag_0.db3 --config ./src/lightning-lm/config/default_deep_robotics.yaml
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
### rviz2 for Real-time Visualization
Although the original Pangolin interface is more efficient, rviz2 visualization is also provided.

The rviz display includes tf, Odometry, PointCloud2-currentScan, and PointCloud2-globalMap. The currentScan is transformed to the global 'map' frame and processed by Undistortion and Downsampling.
```bash
rviz2 -d src/lightning-lm/config/showbodypc.rviz
```
If `system.pub_tf: true`, the `/lightning/odom` odometry topic will be visible here.

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
        ros2 run lightning run_loc_online --config ./src/lightning-lm/config/default_deep_roboticsloc.yaml
        ```

### Option B: Offline Localization
Run localization on a bag file without real-time constraints to verify algorithm performance.
```bash
ros2 run lightning run_loc_offline --config ./src/lightning-lm/config/default_deep_roboticsloc.yaml --input_bag [path_to_bag]
```

### rviz2 for Real-time Visualization
The rviz display includes tf, Odometry, PointCloud2-currentScan, PointCloud2-globalMap. The global map is set by the input file, which remains constant during online operation.
```bash
rviz2 -d src/lightning-lm/config/showglobalmap.rviz
```