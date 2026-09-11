# Lightning-LM

![Seven 3D reconstructions](doc/images/seven-datasets-overview.png)

3D LiDAR mapping and localization for **Deep Robotics M20 Pro** and **Livox Mid360**.
Build maps with loop closure, view reconstruction live, and localize against saved
point clouds. Both sensor presets use the same four online and offline applications.

## 1. Install and build

Supports **Ubuntu 20.04 / ROS 2 Foxy** and **Ubuntu 22.04 / ROS 2 Humble**.
Use an OpenGL desktop for visualization.
Run these commands from the repository root in a fresh Bash terminal:

```bash
source /opt/ros/humble/setup.bash
bash scripts/install_dep.sh
bash scripts/build.sh
source scripts/setup.bash
```

On Ubuntu 20.04, use `source /opt/ros/foxy/setup.bash` in the first line.
Run `source scripts/setup.bash` from the repository root in every new application
or bag-playback terminal.

<details>
<summary>1.1 Platform and build details</summary>

| Platform | ROS distribution |
|---|---|
| Ubuntu 20.04 | Foxy |
| Ubuntu 22.04 | Humble |

The native ROS pairings are [Humble with Ubuntu 22.04](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
and [Foxy with Ubuntu 20.04](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html).
The scripts select dependencies from the sourced ROS distribution and load the
same distribution at runtime. Use a separate build/checkout when switching ROS
versions to keep generated interfaces and libraries consistent.

The installer uses `sudo apt-get` for missing dependencies. The build uses all
CPUs, builds the bundled Pangolin source into `.deps`, and installs the ROS package
locally. On a computer with limited RAM, including the RK3588, start with:

```bash
CMAKE_BUILD_PARALLEL_LEVEL=2 bash scripts/build.sh
```

A clean source build was checked; a fresh operating-system installation was not.
For headless operation, set `system.with_ui: false` in the selected configuration.

</details>

## 2. Select a sensor and recording

| Sensor system | Configuration | LiDAR input | IMU input |
|---|---|---|---|
| M20 Pro | `config/m20_pro.yaml` | `/LIDAR/POINTS` · `sensor_msgs/msg/PointCloud2` | `/IMU` |
| Mid360 | `config/mid360.yaml` | `/livox/lidar` · `livox_ros_driver2/msg/CustomMsg` | `/livox/imu` |

Use the same preset for mapping and localization. Set paths for your recording
and a **new** map directory; `BAG` accepts a SQLite bag directory or `.db3` file:

```bash
export CONFIG="$PWD/config/mid360.yaml"
export BAG="/absolute/path/to/recording"
export MAP="$PWD/data/my_map"
```

For M20 Pro, select `config/m20_pro.yaml`. Check the topic names and sensor
calibration when using another mounting.

<details>
<summary>2.1 Timestamps, calibration, and 3D estimation</summary>

LiDAR and IMU messages may arrive asynchronously and at different rates. Their
timestamps must use a consistent time base: odometry waits for IMU coverage
through each scan's end and uses per-point times to compensate motion. Exact
message pairing or simultaneous arrival is unnecessary. The code does not
estimate sensor clock offsets; correct offsets or drift in the driver/time source.
Hardware clock synchronization is one way to satisfy this requirement, not an
extra trigger mechanism required by the application. Bag playback's `--clock`
does not repair incorrect sensor timestamps.

M20 points require `x`, `y`, `z`, `intensity`, and absolute per-point `timestamp`
in seconds, on the same time base as the scan header. Mid360 uses `offset_time`
in nanoseconds relative to the scan header. IMU units are radians/second and
metres/second².

Extrinsics transform LiDAR points into the IMU frame:
`p_imu = extrinsic_R * p_lidar + extrinsic_T`. The supplied translations are
`[0, 0, 0]` for M20 Pro and `[0, 0, 0.28]` for the tested Mid360 mounting.
The Mid360 value is mounting-specific; calibrate it for your installation.

M20 uses inertial motion prediction. Mid360 uses constant-velocity translation
with gyro rotation. Both estimate full 3D motion, use 0.5 m scan/map voxels, and
disable fixed-height and 2D pose constraints. See the
[implementation notes](doc/implementation.md) for the model and timing details.

</details>

## 3. Build a map

### 3.1 Offline mapping

```bash
ros2 run lightning run_slam_offline \
  --config "$CONFIG" --input_bag "$BAG" --map_path "$MAP"
```

The viewer opens during processing. At the end, the application saves the map,
prints `map saved`, and closes the viewer.

### 3.2 Online mapping

Start the node before the sensor drivers or bag playback:

```bash
ros2 run lightning run_slam_online --config "$CONFIG"
```

In another sourced terminal, set `BAG` and replay the recording:

```bash
ros2 bag play "$BAG" --rate 1.0
```

For live operation, start your sensor drivers using [section 5](#5-connect-the-sensor-drivers).
When mapping is finished, call this service from another sourced terminal:

```bash
ros2 service call /lightning/save_map lightning/srv/SaveMap "{map_id: online_map}"
```

Wait for `response: 0`, then press **Ctrl+C** in the mapping terminal. The map is
saved to `data/online_map/` relative to that terminal's working directory.
Choose a new `map_id` for each run. Online mapping requires this explicit save.

<details>
<summary>3.3 Map files and viewer controls</summary>

Keep the complete map directory: `global.pcd` is the full point cloud;
`index.txt` and numbered `.pcd` tiles are also required for localization.
Nonempty output directories are protected from overwriting. To localize against
the online map, set `MAP="$PWD/data/online_map"` in the localization terminal.

Use the mouse to rotate, pan, and zoom; enable **Follow** to track the robot.
Long routes can extend beyond the initial view. If the viewer is blank, check
that sensor data is arriving, then adjust the camera. Run from an OpenGL desktop
with `DISPLAY` set; setup selects Pangolin's X11 backend, including through XWayland.

For the saved-map viewer below, `-ps 2` sets the point size. Press **h** in the
PCL window for its controls. `pcl-tools` is installed by the dependency script;
see the [PCL viewer reference](https://pointclouds.org/documentation/tutorials/walkthrough.html#binaries)
for additional display options. Opening a PCD in this viewer does not modify it.

</details>

### 3.4 View a saved map

After offline mapping finishes:

```bash
pcl_viewer "$MAP/global.pcd" -ps 2
```

After online mapping returns a successful save response:

```bash
pcl_viewer "$PWD/data/online_map/global.pcd" -ps 2
```

Use your chosen map directory or `map_id` if it differs from these examples.

## 4. Localize in a saved map

### 4.1 Offline localization

```bash
ros2 run lightning run_loc_offline \
  --config "$CONFIG" --input_bag "$BAG" --map_path "$MAP" \
  --trajectory "$PWD/localization.tum"
```

### 4.2 Online localization

```bash
ros2 run lightning run_loc_online \
  --config "$CONFIG" --map_path "$MAP" \
  --trajectory "$PWD/localization_online.tum"
```

Start the drivers or replay the bag from another sourced terminal using the
command in section 3.2. The viewer shows the reference map, scan, and trajectory.
Press **Ctrl+C** after playback to close the viewer and flush the trajectory.

<details>
<summary>4.3 Initialization and pose output</summary>

Initialization starts around the map's saved starting pose. Replaying the mapping
recording needs no manual initial pose. To start elsewhere, publish
`geometry_msgs/msg/PoseWithCovarianceStamped` on `/initialpose` with
`header.frame_id: map`, for example with RViz's **2D Pose Estimate** tool.
This supplies a starting guess; estimation remains 3D.

Online localization publishes `map` → `base_link` on `/tf` and
`geometry_msgs/msg/PoseStamped` on `/lightning/pose`. The optional trajectory
contains valid map matches as `timestamp x y z qx qy qz qw`; an existing trajectory
file is replaced. Localization keeps the reference map unchanged.

Run mapping and localization separately when evaluating their results. Topic
names, extrinsics, and the saved map must match the selected sensor setup.

</details>

## 5. Connect the sensor drivers

### 5.1 M20 Pro

On the NOS host, start the point-cloud relay:

```bash
ssh user@10.21.31.106
sudo systemctl start multicast-relay.service
sudo systemctl status multicast-relay.service
```

In another terminal, connect to AOS through the robot's Wi-Fi:

```bash
ssh user@10.21.41.1
```

Change to the repository directory built in section 1, then prepare a root shell:

```bash
sudo -s
source /opt/robot/scripts/setup_ros2.sh
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/config/fastdds.xml"
source scripts/setup.bash
export CONFIG="$PWD/config/m20_pro.yaml"
```

Check each sensor topic, stopping each command with **Ctrl+C**:

```bash
ros2 topic hz /LIDAR/POINTS
ros2 topic hz /IMU
```

Keep the firmware sensor publishers running. Use the online mapping command in
section 3.2. In a second AOS terminal, repeat the root-shell setup above and call
the save service. Wait for `response: 0` before stopping mapping:

```bash
ros2 service call /lightning/save_map lightning/srv/SaveMap "{map_id: online_map}"
```

For localization, load that map and give Lightning-LM its own TF topic alongside
the firmware's localization output:

```bash
export MAP="$PWD/data/online_map"
ros2 run lightning run_loc_online \
  --config "$CONFIG" --map_path "$MAP" \
  --trajectory "$PWD/localization_online.tum" \
  -- --ros-args -r /tf:=/lightning/tf -r /initialpose:=/lightning/initialpose
```

Pose output is `/lightning/pose`; transforms are on `/lightning/tf`. Use a new
`map_id` for subsequent mapping runs and update `MAP` to match.

<details>
<summary>5.1.1 Connection and headless operation</summary>

AOS is also reachable at `10.21.33.103` on the robot's internal network.
To start the relay automatically after reboot, run
`sudo systemctl enable multicast-relay.service` on NOS.

For an SSH session without a display, make a runtime copy of the M20 preset with
the viewer disabled. Use this `CONFIG` for both mapping and localization:

```bash
mkdir -p data
cp config/m20_pro.yaml data/m20_pro_headless.yaml
sed -i 's/with_ui: true/with_ui: false/' data/m20_pro_headless.yaml
export CONFIG="$PWD/data/m20_pro_headless.yaml"
```

Repeat the `CONFIG` assignment in each application terminal. View the saved
`global.pcd` on a computer with a display using section 3.4. Preserve the robot's
sensor clock synchronization when setting up live inputs.

</details>

### 5.2 Mid360

Start the hardware driver from its own workspace. This repository includes Livox
message definitions; install and launch the full Mid360 driver separately.
In the driver's sourced terminal, set the following before launching it, using
this repository's location:

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="/path/to/lightning-lm-deep-robotics/config/fastdds.xml"
```

Keep the driver's existing reliability setting. Lightning-LM uses **best-effort**
subscriptions and accepts either best-effort or reliable sensor publishers.

<details>
<summary>5.3 Best-effort DDS and the larger shared-memory buffer</summary>

Best effort avoids waiting for retransmission of missing samples. Adequate
processing capacity and buffering still matter; delivery is not guaranteed.
A reliable Mid360 publisher does **not** need to be changed to best effort.
A best-effort publisher cannot satisfy a reliable subscriber, so both supplied
presets retain `common.sensor_qos: best_effort`.
See the [ROS 2 QoS compatibility rules](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html#qos-compatibilities).

`scripts/setup.bash` selects Fast DDS and loads `config/fastdds.xml`, increasing
the shared-memory segment to **64 MiB per participant**, with an **8 MiB** maximum
message and **4096** queued descriptors. Some M20 scans approach 4 MiB; Fast DDS
2.6's default 512 KiB segment can be too small for one scan. This expands transport
capacity while preserving best-effort QoS. See the
[Fast DDS buffer documentation](https://fast-dds.docs.eprosima.com/en/2.6.x/fastdds/transport/shared_memory/shared_memory.html).

Apply the profile to the application and publisher, including the bag player.
Restart an existing driver after applying it: outgoing buffers belong to the
publisher. Keep the driver's own workspace sourced; sourcing Lightning-LM's
overlay there can shadow the full `livox_ros_driver2` package with message-only
definitions. Explicit middleware/profile environment overrides are preserved by
setup and must select Fast DDS and this XML to use these settings.

Shared memory applies on the same computer. UDP remains enabled between
computers, where network and socket capacity need separate validation.
If playback falls behind, reduce `--rate`. Larger queues cannot fix sustained
CPU overload.

</details>

## 6. Results and onboard resources

<details>
<summary>6.1 Seven-dataset reconstruction and localization results</summary>

All seven recordings passed offline/online mapping and offline/online localization
checks on Ubuntu 22.04 / Humble, with the viewer enabled and online playback at 1×.
Each sensor family uses one preset without route-specific tuning.

| Recording | Configuration | Offline mapping | Online mapping | Offline localization | Online localization |
|---|---|---|---|---|---|
| Library F | `m20_pro.yaml` | Pass | Pass | Pass | Pass |
| Office | `m20_pro.yaml` | Pass | Pass | Pass | Pass |
| Building 1 | `mid360.yaml` | Pass | Pass | Pass | Pass |
| Building 2 | `mid360.yaml` | Pass | Pass | Pass | Pass |
| Building 3 | `mid360.yaml` | Pass | Pass | Pass | Pass |
| Grass 2 | `mid360.yaml` | Pass | Pass | Pass | Pass |
| Road 1 | `mid360.yaml` | Pass | Pass | Pass | Pass |

Checks include input counts, rendered viewers, saved point clouds, valid
localization trajectories, pose/TF publication, and preservation of reference
maps. One preceding best-effort run missed one IMU sample; its unchanged repeat
passed. Library F retains visible revisit misalignment. These are operational
and qualitative reconstruction results, without surveyed ground-truth accuracy.
See [validation details and recording paths](doc/validation.md) for the evidence.

</details>

<details>
<summary>6.2 RK3588 and the 16 GB memory budget</summary>

The largest measured application memory footprint was **1.26 GiB**, or **1.41 GiB**
including bag playback, on the x86 workstation with visualization enabled.
These sampled sums of process RSS exclude the display server, OS, and other robot
software; shared pages may be counted twice. RK3588 runtime and memory still
require hardware testing.

Point sampling remains every sixth M20 point or every fourth Mid360 point, with
0.5 m voxels. Map localization is capped at 5 Hz while incoming scans and IMU
samples continue through odometry. NDT and parallel point processing each use a
four-worker limit; OpenMP defaults to passive waiting. Sensor queues are bounded,
and localization unloads distant map tiles. Mapping memory grows with route length
because loop closure retains keyframes.

Start onboard builds with two jobs and validation at normal sensor rate. Disable
`system.with_ui` for headless operation. If map matching cannot keep up, lower
`lidar_loc.max_frequency` and recheck tracking. See
[resource and implementation details](doc/implementation.md).

</details>

## 7. Maintenance and license

Contributor guidance lives in [AGENTS.md](AGENTS.md), with detailed
[implementation](doc/implementation.md) and [validation](doc/validation.md) notes.

Based on [Lightning-LM](https://github.com/gaoxiang12/lightning-lm).
See [LICENSE.txt](LICENSE.txt) for the BSD 3-Clause license. Bundled dependencies
retain their own licenses.
