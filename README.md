# Lightning-LM

![Seven 3D reconstructions](doc/images/seven-datasets-overview.png)

3D LiDAR mapping and localization for **Deep Robotics M20 Pro** and **Livox Mid360**.
Build maps with loop closure, view reconstruction live, and localize against saved
point clouds. Both sensor presets use the same four online and offline applications.

## 1. Install and build

Supports **Ubuntu 20.04 / ROS 2 Foxy** and **Ubuntu 22.04 / ROS 2 Humble**.
Use an OpenGL desktop for visualization. For an M20 Pro without internet access,
follow [onboard deployment](#5-deploy-from-a-laptop-to-the-robot).
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
locally. On a workstation with limited RAM, reduce the job count:

```bash
CMAKE_BUILD_PARALLEL_LEVEL=2 bash scripts/build.sh
```

Keep the generated build directories and rerun `bash scripts/build.sh` after
editing the code. The installer includes `ccache`, which the build script uses
automatically to reuse previous compilations. Run `ccache -s` to see cache usage.

The default Release build omits debug symbols. To include them for debugging:

```bash
CMAKE_BUILD_TYPE=RelWithDebInfo bash scripts/build.sh
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

## 3. Map a recording

### 3.1 Offline mapping

```bash
ros2 run lightning run_slam_offline \
  --config "$CONFIG" --input_bag "$BAG" --map_path "$MAP"
```

The viewer opens during processing. At the end, the application saves the map,
prints `map saved`, and closes the viewer.

### 3.2 Online mapping

Start the node before bag playback:

```bash
ros2 run lightning run_slam_online --config "$CONFIG"
```

In another sourced terminal, set `BAG` and replay the recording:

```bash
ros2 bag play "$BAG" --rate 1.0
```

When mapping is finished, call this service from another sourced terminal:

```bash
ros2 service call /lightning/save_map lightning/srv/SaveMap "{map_id: online_map}"
```

Wait for `response: 0`, then press **Ctrl+C** in the mapping terminal. The map is
saved to `data/online_map/` relative to that terminal's working directory.
Choose a new `map_id` for each run. Online mapping requires this explicit save.
For live sensor input, follow [onboard mapping](#7-onboard-mapping).

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

## 4. Localize a recording

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

Replay the bag from another sourced terminal using the
command in section 3.2. The viewer shows the reference map, scan, and trajectory.
Press **Ctrl+C** after playback to close the viewer and flush the trajectory.
For live sensor input, follow [onboard localization](#8-onboard-localization).

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

## 5. Deploy from a laptop to the robot

Download the repository on your laptop, transfer the sources to AOS, and compile
and install there. This uses the standard M20 Pro AOS environment with
Ubuntu 20.04 / Foxy; AOS does not need an external internet connection.

### 5.1 Laptop: download and transfer the sources

Download this repository using **Code → Download ZIP** and extract it on your
laptop, or use your existing Git checkout. Open a Bash terminal in its root
and create a source archive:

```bash
tar -czf /tmp/lightning-source.tar.gz \
  CMakeLists.txt package.xml cmake config scripts src srv \
  thirdparty/Pangolin-0.9.3.zip thirdparty/Sophus thirdparty/livox_ros_driver \
  README.md AGENTS.md LICENSE.txt doc
```

Connect the laptop to the robot network. Set `AOS` to the robot's address and
transfer the archive:

```bash
export AOS=10.21.33.103
scp /tmp/lightning-source.tar.gz "user@$AOS:~/"
ssh "user@$AOS"
```

When using the additional network adapter, use `export AOS=10.21.41.1` instead.

### 5.2 AOS: compile and install Lightning-LM

In the SSH terminal, extract into a new deployment directory and build:

```bash
mkdir -p ~/lightning-lm
tar -xzf ~/lightning-source.tar.gz --touch -C ~/lightning-lm
cd ~/lightning-lm
source /opt/ros/foxy/setup.bash
bash scripts/build_robot.sh
source scripts/setup.bash
ros2 pkg executables lightning
ros2 interface show lightning/srv/SaveMap
ros2 interface show livox_ros_driver2/msg/CustomMsg
```

The script checks the installed dependencies, compiles the bundled Pangolin and
Lightning-LM sources, and installs them in this deployment directory. It selects
up to four compiler jobs according to available RAM.

Continue with [sensor preparation](#61-m20-pro),
[onboard mapping](#7-onboard-mapping), and
[onboard localization](#8-onboard-localization).

<details>
<summary>5.3 Build settings and subsequent code updates</summary>

On RK3588, the robot build script distributes compiler jobs across the four A76
cores and runs compilation at reduced scheduling priority. It reduces the default
job count when less RAM is available. This is an initial memory estimate; other
software can still consume RAM during the build. Runtime threading is configured
separately.

To check dependencies and see the selected settings without building:

```bash
bash scripts/build_robot.sh --check
```

Keep `build`, `build-pangolin`, `.deps`, and `install` for subsequent builds.
After transferring changed sources, rebuild from the same AOS directory:

```bash
bash scripts/build_robot.sh
source scripts/setup.bash
```

Unchanged files are skipped. If `ccache` is installed, it also reuses previous
compilations when their inputs match. To request fewer jobs:

```bash
CMAKE_BUILD_PARALLEL_LEVEL=2 bash scripts/build_robot.sh
```

The source archive works with a downloaded ZIP or Git checkout and includes
local source edits and all bundled source dependencies. Laptop build products
are excluded; AOS produces native ARM binaries. The `--touch` extraction option
gives transferred source files AOS's current modification time. Preserve the
robot's sensor clock synchronization and existing build-product timestamps.

</details>

## 6. Connect the sensor drivers

### 6.1 M20 Pro

On the NOS host, start the point-cloud relay:

```bash
ssh user@10.21.31.106
sudo systemctl start multicast-relay.service
sudo systemctl status multicast-relay.service
```

On a Linux desktop, connect to AOS with X11 forwarding:

```bash
ssh -Y -C user@10.21.33.103
```

When using the additional network adapter configured for `10.21.41.1`, connect
through that address instead:

```bash
ssh -Y -C user@10.21.41.1
```

On Windows, use an SSH client with an X server, such as MobaXterm, and enable
**X11 forwarding** for the SSH session. Lightning-LM does not require a particular
SSH client.

Complete [onboard deployment](#5-deploy-from-a-laptop-to-the-robot), then change
to that repository directory on AOS and prepare a root shell:

```bash
sudo env DISPLAY="$DISPLAY" XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}" bash
source /opt/robot/scripts/setup_ros2.sh
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/config/fastdds.xml"
source scripts/setup.bash
```

Prepare a runtime copy that disables the built-in map viewer. The onboard
commands below enable the separate RViz2 view:

```bash
mkdir -p data
cp config/m20_pro.yaml data/m20_pro_headless.yaml
sed -i 's/with_ui: true/with_ui: false/' data/m20_pro_headless.yaml
export CONFIG="$PWD/data/m20_pro_headless.yaml"
```

Check each sensor topic, stopping each command with **Ctrl+C**:

```bash
ros2 topic hz /LIDAR/POINTS
ros2 topic hz /IMU
```

Keep the firmware sensor publishers running. Continue with
[onboard mapping](#7-onboard-mapping) or
[onboard localization](#8-onboard-localization).
Repeat the root-shell setup in every AOS application or service terminal.

<details>
<summary>6.1.1 Relay and runtime configuration</summary>

To start the relay automatically after reboot, run
`sudo systemctl enable multicast-relay.service` on NOS.

The runtime copy changes only `system.with_ui`; sensor calibration and estimator
settings remain the same as `config/m20_pro.yaml`. Use the same copy for mapping
and localization. Preserve the robot's sensor clock synchronization when setting
up live inputs.

The root-shell command preserves SSH's display and X authorization. Keep the
SSH-assigned `DISPLAY`; do not replace it with the laptop's IP address. RViz2 and
its ROS subscriptions run on AOS, while SSH forwards the window to the laptop.
The map remains onboard. Window size and refresh rate still affect Wi-Fi traffic.

If the window cannot open, check `echo "$DISPLAY"` before entering the root shell
and reconnect with X11 forwarding enabled. The RViz command uses software OpenGL
for forwarded displays. MobaXterm also provides OpenGL settings under
**Settings → Configuration → X11**; see its
[X11 documentation](https://mobaxterm.mobatek.net/documentation.html).

</details>

### 6.2 Mid360

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
<summary>6.3 Best-effort DDS and the larger shared-memory buffer</summary>

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

## 7. Onboard mapping

Complete [M20 Pro preparation](#61-m20-pro), then run these commands from the
repository root on AOS. Keep the robot standing during initialization and leave
the sensor publishers running.

### 7.1 Start mapping

In the prepared root shell:

```bash
export CONFIG="$PWD/data/m20_pro_headless.yaml"
taskset -c 7 ros2 run lightning run_slam_online --config "$CONFIG" --rviz
```

### 7.2 View the location, LiDAR, and trajectory

In another X11-forwarded AOS terminal, change to the repository root and repeat
the root-shell setup in section 6.1, then open RViz2:

```bash
LIBGL_ALWAYS_SOFTWARE=1 LP_NUM_THREADS=2 QT_X11_NO_MITSHM=1 \
  taskset -c 6 rviz2 -d config/onboard.rviz \
  --ros-args -r /tf:=/lightning/tf -r /tf_static:=/lightning/tf_static
```

The window shows the **current LiDAR scan**, a **red location arrow**, and the
**yellow trajectory**. The full map is not displayed. You can open RViz2 after
mapping starts and still see the trajectory recorded since startup.

### 7.3 Save the map

In a second AOS terminal, repeat the root-shell setup in section 6.1, then save
to a new map directory:

```bash
ros2 service call /lightning/save_map lightning/srv/SaveMap "{map_id: onboard_map}"
```

Wait for `response: 0` before pressing **Ctrl+C** in the mapping terminal.
The map is saved to `data/onboard_map/` relative to the mapping terminal's
repository root. Choose a new `map_id` for each mapping run.

<details>
<summary>7.4 Live view, CPU placement, and saved maps</summary>

Onboard mapping uses the same `run_slam_online` application and M20 sensor
settings as online dataset playback. The robot's firmware supplies LiDAR and IMU
messages directly; no bag player is needed.

Keep the complete map directory, including `index.txt`, all numbered `.pcd`
tiles, and `global.pcd`. The same map format is used by recorded-data and onboard
localization. The runtime copy disables Pangolin; `--rviz` enables the lightweight
ROS display outputs. Neither the RViz preset nor these outputs includes a map
cloud.

On the standard RK3588 firmware, the commands place estimation on A76 core 7 and
RViz2 on A76 core 6. This avoids the A55 affinity inherited by some SSH shells and
keeps rendering separate from estimation. Compiler job settings do not set runtime
affinity. These CPU numbers apply to the M20 Pro AOS hardware.

The current scan is deskewed and uses the same LiDAR pose and acquisition timestamp
as the arrow. Both update at up to 5 Hz. Only the newest scan is shown; the path
retains the whole session at this sample rate and refreshes at 1 Hz. Opening RViz2
late or reopening it retrieves the retained path while the application is running.
Restarting the application starts a new path. Path memory grows with session length.

Mapping displays the current pose with the latest keyframe correction. Historical
trajectory points keep their estimates from when they were recorded. The view
follows `lightning_lidar`. To inspect the whole route, change **Target Frame** to
`map` and zoom out; rotate and pan with the mouse. RViz's **Frame Rate** is set to 5
to limit remote redraws. Actual rendering can be slower depending on the display
and connection; reduce the setting or shrink the window on a slower connection.

</details>

### 7.5 View a saved map separately

On a computer with a display, copy the complete map directory into
`data/onboard_map/` if needed, then open the saved cloud:

```bash
pcl_viewer data/onboard_map/global.pcd -ps 2
```

## 8. Onboard localization

Stop mapping first. Complete [M20 Pro preparation](#61-m20-pro) in the
localization terminal and run from the repository root on AOS.

### 8.1 Load the map and start localization

Use the map saved in section 7, or set `MAP` to an existing complete map directory:

```bash
export CONFIG="$PWD/data/m20_pro_headless.yaml"
export MAP="$PWD/data/onboard_map"
taskset -c 7 ros2 run lightning run_loc_online \
  --config "$CONFIG" --map_path "$MAP" --rviz \
  --trajectory "$PWD/localization_onboard.tum" \
  -- --ros-args -r /tf:=/lightning/tf -r /initialpose:=/lightning/initialpose
```

### 8.2 View the location, LiDAR, and trajectory

Close the mapping RViz window. In another X11-forwarded AOS terminal, change to
the repository root and repeat the root-shell setup in section 6.1, then run:

```bash
LIBGL_ALWAYS_SOFTWARE=1 LP_NUM_THREADS=2 QT_X11_NO_MITSHM=1 \
  taskset -c 6 rviz2 -d config/onboard.rviz \
  --ros-args -r /tf:=/lightning/tf -r /tf_static:=/lightning/tf_static
```

The same view shows the current scan, location arrow, and full localization
trajectory. The reference map is used onboard without displaying it in RViz2.

### 8.3 Stop localization

Press **Ctrl+C** in the localization terminal when finished, then close RViz2.
Valid localization poses are written to `localization_onboard.tum`; the reference
map is preserved.

<details>
<summary>8.4 Initialization, dataset maps, and pose output</summary>

Onboard localization uses the same `run_loc_online` application and sensor
settings as online dataset playback. Maps from recorded data and live mapping
use the same directory format; keep the complete tiles and use the matching
sensor calibration.

Initialization starts around the map's saved starting pose. Start near that
pose, or provide an initial estimate on `/lightning/initialpose` using
`geometry_msgs/msg/PoseWithCovarianceStamped` with `header.frame_id: map`. The
RViz **2D Pose Estimate** tool is configured for this topic; it supplies a starting
guess and does not constrain subsequent estimation to 2D.

The RViz scan, arrow, and path use accepted scan-to-map matches. Before
initialization succeeds they remain empty; if matching fails they retain the last
valid display until a new match succeeds. Check the localization terminal when
the display stops updating.

The robot's firmware already publishes transforms on `/tf`. The command above
publishes Lightning-LM's `map` → `base_link` transform on `/lightning/tf` and
receives initial poses on `/lightning/initialpose`. Pose output remains on
`/lightning/pose`. The `--` separator before `--ros-args` is required.

The display topics are `/lightning/current_pose`, `/lightning/current_scan`, and
`/lightning/trajectory`, in frame `map`. The separate visualization transform is
`map` → `lightning_lidar` on `/lightning/tf`. Existing `/lightning/pose` output
continues to provide the estimator's higher-frequency pose.

The trajectory file uses `timestamp x y z qx qy qz qw` rows and includes valid map
matches. An existing trajectory file at the selected path is replaced.

</details>

## 9. Results and onboard resources

<details>
<summary>9.1 Seven-dataset reconstruction and localization results</summary>

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
<summary>9.2 RK3588 and the 16 GB memory budget</summary>

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

Use `scripts/build_robot.sh` for onboard builds and validate at normal sensor rate. Disable
`system.with_ui` for headless operation. If map matching cannot keep up, lower
`lidar_loc.max_frequency` and recheck tracking. See
[resource and implementation details](doc/implementation.md).

</details>

## 10. Maintenance and license

Contributor guidance lives in [AGENTS.md](AGENTS.md), with detailed
[implementation](doc/implementation.md) and [validation](doc/validation.md) notes.

Based on [Lightning-LM](https://github.com/gaoxiang12/lightning-lm).
See [LICENSE.txt](LICENSE.txt) for the BSD 3-Clause license. Bundled dependencies
retain their own licenses.
