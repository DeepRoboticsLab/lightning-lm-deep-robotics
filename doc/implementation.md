# Implementation and deployment notes

## 1. Runtime layout

| Component | Source | Responsibility |
|---|---|---|
| Applications | `src/app/run_*.cc` | Arguments, configuration, lifecycle, four-way oneTBB limit |
| Mapping system | `src/core/system/slam.cc` | Offline callbacks, online sensor queue, viewer, loop closure, map-save service |
| Localization system | `src/core/system/loc_system.cc` | ROS input, initial pose, pose/TF output, shutdown |
| Bag reader | `src/wrapper/bag_io.{h,cc}` | SQLite/CDR reading and native message deserialization |
| SLAM input recorder | `src/wrapper/slam_recorder.{h,cc}`, `msg/SlamInput.msg` | Bounded asynchronous recording of synchronized inputs and exact-value debug replay |
| LiDAR odometry | `src/core/lio/laser_mapping.cc` | Input pairing, ESKF update, local point-to-plane registration, keyframes |
| Preprocessing / deskew | `src/core/lio/pointcloud_preprocess.cc`, `imu_processing.hpp` | Sensor-specific point times and motion compensation |
| Filter | `src/core/lio/eskf.{hpp,cc}`, `src/common/nav_state.h` | State propagation, covariance, iterated measurement update |
| Loop closure | `src/core/loop_closing/loop_closing.cc`, `src/core/miao` | Candidate registration and pose graph optimization |
| Map localization | `src/core/localization/localization.cpp`, `lidar_loc/lidar_loc.cc` | Odometry prediction, voxelized NDT matching, accepted poses |
| Tiled maps | `src/core/maps/tiled_map*.{h,cc}` | Map index, loading, unloading, point cloud export |
| Input queue | `src/utils/async_message_process.h` | Ordered work, count/byte limits, overflow reporting, drain |
| Terminal status | `src/utils/console.{h,cc}` | TTY refresh, plain summaries, durable events and detailed diagnostic logs |
| Visualization | `src/ui/pangolin_window*.{h,cc}`, `ui_cloud.cc` | Render thread, viewport, clouds, trajectory, camera controls |

Paths in the localization row are relative to `src/core/localization/`. Source
headers use `.hpp` where present; use `rg --files src` to locate exact files.

The four estimator entry points start a terminal session before their system
objects and finish it after sensor/loop workers have stopped. A glog sink keeps
routine diagnostics in a per-session file while showing warnings/errors with
source-based repeat counts. Map-save events remain in scrollback. Status is
updated at 1 Hz on a capable TTY or every 10 seconds in plain output; verbose mode
uses ordinary glog terminal output. Metrics use a separate mutex from terminal
writes, with atomic input counters, so a slow display cannot hold their lock.
Rates are counts per steady-clock interval, not acquisition-clock frequency.
An ordinary wait for IMU coverage is an INFO diagnostic, not a synchronization
failure. The IMU initializer also normally returns without a deskewed scan;
only empty output after initialization warns. Timestamp rejection and
queue-overflow warnings remain intact.
On a TTY, a bounded line reader also coordinates direct ROS/library output with
redraws and writes its original bytes to `external.log`. It captures only file
descriptors attached to that terminal, preserving separately redirected stdout.
Shutdown restores descriptors and drains pending messages before the final
summary; fatal logging restores stderr before glog prints its crash trace.

## 2. Estimation and the two presets

The odometry frontend uses an iterated error-state Kalman filter with 23 error
degrees of freedom: position, orientation, LiDAR–IMU extrinsics, velocity, gyro
bias, acceleration bias, and a two-dimensional gravity direction. LiDAR residuals
match scan points to local planes. A representative residual is
`nᵀ (R_world_imu (R_imu_lidar p_lidar + t_imu_lidar) + t_world_imu) + d`.
The IMU predicts motion and enables scan deskewing; LiDAR updates correct pose.
Loop closure registers revisited keyframes and optimizes the map poses.

For M20, propagation integrates gyro and acceleration:
`p_dot = v`, `R_dot = R [omega - b_g]x`, `v_dot = R (a - b_a) + g`.
Mid360's selected motion model keeps `p_dot = v` and gyro rotation but sets
`v_dot = 0`. Its translational Jacobian removes acceleration, orientation,
acceleration-bias, and gravity coupling consistently with that mean; the
acceleration-bias random-walk block is disabled while velocity uncertainty is
retained. LiDAR can still update velocity and translation in all three axes.
This model does not lock height or remove the need for IMU timestamps/gyro data.

| Effective setting | M20 Pro | Mid360 |
|---|---|---|
| File | `config/m20_pro.yaml` | `config/mid360.yaml` |
| LiDAR type | 3 / RoboSense PointCloud2 | 1 / Livox CustomMsg |
| Motion model | `inertial` | `constant_velocity` |
| IMU filtering | true | false |
| Point sampling | every 6th | every 4th |
| Blind distance | 0.5 m | 0.1 m |
| Scan / map voxel | 0.5 m / 0.5 m | 0.5 m / 0.5 m |
| LIO maximum iterations | 4 | 4 |
| Keyframe distance / angle | 2 m / 15° | 1 m / 10° |
| Extrinsic translation | `[0,0,0]` | `[0,0,0.28]` |
| Extrinsic rotation | identity | identity |
| Loop keyframe gap / minimum ID interval | 20 / 20 | 20 / 20 |
| Closest ID threshold / loop range | 20 / 60 m | 50 / 20 m |
| Loop NDT score threshold | 0.6 | 0.6 |
| Optimize on every keyframe | true | false |

Extrinsics use `p_imu = R_imu_lidar p_lidar + t_imu_lidar`. The Mid360 translation
belongs to the reference mounting, not the product specification. Calibrate a new
installation; do not change these values as a cosmetic reconstruction fix.

Both presets disable Anderson acceleration (`use_aa`), fixed-height loop priors,
2D localization projection, and g2p5 output. They enable loop closure and the 3D
viewer. Preserve the six pose degrees of freedom across slopes and multi-level
scenes. Global map tilt alone is not evidence that a height constraint is needed.

The recording presets keep `fasterlio.skip_lidar_num: 0`. New default onboard
runtime copies use `2` for both M20 and AGX: after initialization, every second
scan reaches voxel filtering, LiDAR correction and incremental mapping. IMU
prediction and deskewing still run on each scan before the skip decision. At
10 Hz input this gives approximately 5 Hz LiDAR corrections, independent of the
localization map-matching and RViz publication limits. The launcher preserves
existing runtime YAML and explicit `--config` sampling choices. Sensor calibration
is unchanged apart from the documented AGX internal-IMU transform.

### Loop rejection and retries

Miao's incremental path reuses vertex and Hessian-block allocation, but rebuilds
the active edge list on each initialization. Changing an edge's level must affect
both the cost and Hessian in subsequent solves; retaining the edge in the graph
does not make it active. All allocated Hessian blocks are cleared before each
linearization. The mapping backend marks outliers inactive, restores the estimates
from before that solve, and solves again before writing optimized keyframe poses.
This removes rejected constraints without retaining their free-gauge displacement.
It does not establish that every surviving loop is geometrically correct.

Successful loops retain the `loop_kf_gap` cooldown. Failed attempts instead use
`loop_retry_kf_gap` (default five new keyframes), bounding extra registration work
while allowing another attempt on approach to a revisit. Candidates still depend
on odometry proximity and NDT alignment. A stopped robot creates no new distance
keyframes, so waiting alone cannot force recognition.

## 3. Sensor timing and synchronization

Pose lookup accepts an exact first sample, including when the queue contains
only that sample. Interpolation or extrapolation at a different time still
requires the existing history and time limits. This avoids rejecting the first
localization/PGO frame when scan-end odometry is already available at precisely
the requested timestamp.

The separate IMU prediction filter is seeded from the first initialized scan,
as it is after later LiDAR corrections. Until that seed exists its pose is not
available to DR consumers. Localization inserts the exact scan anchor before
its first match, then receives subsequent predictions through IMU callbacks.
Prediction advances from its own timestamp to the next native IMU timestamp.
After a correction it starts at scan end, avoiding a second integration of the
partial interval between the preceding IMU sample and scan end. This corrects
the separate DR predictor; the scan deskewing and LiDAR correction are unchanged.

`LaserMapping::SyncPackages()` buffers LiDAR and IMU independently. It computes
the scan end from the scan header plus the last retained point's relative time
(or the mean scan duration for sparse/short scans), waits until the latest IMU
timestamp covers that end, and consumes IMU samples through it. Deskewing sorts
points by relative time and propagates/interpolates motion to the scan-end frame.
The model needs temporal alignment of the measurements, not exact ROS message
pairing or simultaneous callback execution.

M20's PointCloud2 `timestamp` is absolute seconds. The preprocessor subtracts
the scan header and multiplies by 1000 to obtain milliseconds. Livox
`offset_time` is divided by 1,000,000 to obtain milliseconds; the scan header is
the time origin. `CustomMsg.timebase` is not substituted for a wrong header.
IMU uses `header.stamp`, angular velocity in rad/s, and acceleration in m/s².

Both streams must describe the same physical time base. There is no estimated
clock offset/drift or generic automatic synchronization service here. Integrated
sensor timestamps can satisfy this without a separate hardware trigger. Sensors
on independent clocks need a verified common clock or timestamp correction in
their driver/time source. Synchronizing host arrival times or publishing `/clock`
does not correct wrong acquisition times. Playback rate changes scheduling while
leaving stored sensor timestamps intact.

Rejecting a backwards LiDAR header must preserve cloud/time queue pairing.
Office and Library F contain 53 and 191 such headers respectively. Count received
messages separately from processed scans. Online localization also retries a
waiting LiDAR scan when IMU arrives; map matching uses the actual LIO scan-end
timestamp rather than a substituted callback timestamp.

### Optional mapping input recording

`--record_bag` snapshots synchronized input groups immediately before the shared
IMU/deskew path. Point selection and range filtering have already run, while
deskewing, voxel filtering, LiDAR corrections and keyframe creation remain part
of replay. Capture must copy values before `ImuProcess` sorts and modifies the
input cloud. Initialization and correction-skipped scans are necessary inputs.
Future/unpaired samples and IMUs used only by the separate display predictor are
not included. This avoids a second subscriber with a different DDS loss pattern.

`SlamInput` stores float XYZ/intensity arrays, double point offsets and IMU values,
exact scan bounds, format version and sequence ID. The ordinary SQLite/CDR bag
has one synchronized-group message per consumed scan. Its timestamp is scan end,
increased by one nanosecond only when needed to preserve sequence ordering; the
native double acquisition times inside the message remain unchanged. Debug replay
uses those values and groups through `RunSynchronized`, without running point
selection or synchronization a second time. It is an offline mapping input format,
not a replacement for a raw driver bag or a reproduction of thread scheduling.

An independent disk thread serializes/writes a bounded queue with a 64 MiB input
budget, including the in-flight group. The ROS bag writer's additional cache is
disabled. Serialization, SQLite and OS buffers add memory beyond that budget.
The producer never waits on storage. Overflow or a write failure stops recording
explicitly while SLAM continues; there is no eviction, compression or adaptive
downsampling. `recording.yaml` remains unfinished until shutdown drains accepted
inputs and finalizes the bag. Consumed/written counts establish recorder coverage,
not completeness of upstream transport. Session logs retain transport failures.
The saved runtime YAML and `--replay_recording` command are described in the README.

## 4. Map localization

Dynamic map files are optional and read only when their loading flag is enabled.
An absent dynamic tile initializes an empty in-memory layer, including after
unloading/revisiting. Static tiles and unreadable enabled dynamic files fail
explicitly. A failed PCD read never marks a tile successfully loaded.

Native sensor preprocessing feeds odometry in localization too. Each completed
LIO update feeds local motion/pose-graph prediction; sensor timestamps throttle
map matching to `lidar_loc.max_frequency: 5.0`. The source cloud is voxelized to
0.5 m before map registration. This is independent of the incoming scan rate.

Fine NDT uses 1 m cells, transformation epsilon 0.01, step 0.1, and 20 iterations.
Coarse recovery uses 5 m cells, epsilon 0.05, step 0.5, and 20 iterations. Both use
four OpenMP threads. First-pose initialization tries the saved map pose before
broader yaw search; broad search is limited to once per two sensor seconds.
Minimum initialization confidence is 1.8; tracking defaults to 1.0. The NDT score
is a registration diagnostic, not a probability or ground-truth accuracy measure.

Registration must converge with finite pose/score and adequate confidence.
Unsuccessful matches are logged and excluded from the TUM export. Loading new
tiles updates the NDT target synchronously so a match cannot silently use the
old region. Static tiles outside the unload radius are released and can reload
on revisit (`load_map_size: 2`, `unload_map_size: 4`).

Reference maps remain read-only: dynamic loading/updating/saving is disabled in
both presets. The optional trajectory is replaced on start and flushed on clean
shutdown. Its rows are `timestamp x y z qx qy qz qw` for valid map matches.
Online pose output uses `/lightning/pose`, with `map` → `base_link` TF.
`/initialpose` accepts a map-frame pose guess; the estimator stays 3D afterward.

### 4.1 Optional global initialization

`--global_init` selects place recognition before ordinary localization. Mapping
writes `places.bin` from its retained keyframe scans and the same poses used to
assemble `global.pcd`. The 20-ring, 60-sector descriptors store maximum local
height within 80 m, after 0.5 m voxelization. Circular sector shifts propose
heading; saved keyframe poses propose position, roll and pitch. NDT then refines
all six pose components. This is not an exhaustive search of arbitrary sensor
roll and pitch.

Descriptor similarity normalizes each populated sector and divides by the union
of populated sectors. Dividing only by their intersection can give a strong
score to a small accidental overlap. Ten spatially distinct proposals are checked
with the existing full-3D NDT implementation. A candidate needs descriptor score
at least 0.55 and NDT confidence at least 2.2 (also respecting the configured NDT
initialization threshold). A competing accepted pose at least 3 m or 20 degrees
away with at least 90% of its NDT score rejects the result. These are rejection
heuristics, not a proof of globally unique alignment.

Three successive candidates must agree with scan-end LiDAR odometry within 1 m
and 10 degrees. Queries are at least 0.5 sensor seconds apart. Online search uses
one worker, one owned scan, a separate map matcher and at most one outstanding
result. It does not queue every scan for recognition. The result is propagated
from its original scan using LiDAR-frame odometry and revalidated on the current
scan before publication. Offline search is synchronous for reproducibility.
Manual initial poses override search; global mode cannot fall back to the map
origin. Its extra matcher/index are released after initialization or a manual
override once the worker finishes. Candidate refinement does not accumulate the
ordinary initializer's failed-pose history or announce tracking success.
Shutdown joins the worker before releasing its data.

The versioned binary index contains explicit scalar fields, an endian marker,
validated poses/descriptors and an FNV-1a fingerprint of `global.pcd` to detect
accidentally mixed maps. It supports up to 20,000 keyframes; mapping beyond this
still exports its map but reports that no place index was written. Old maps remain
usable through the original/manual initialization. Index loading checks size,
version, finite values and the cloud fingerprint; the fingerprint is not a
security signature. Index construction does not alter map points or calibration.

The existing IMU initializer estimates gyro bias from its initial sample mean.
Starting during a turn can therefore corrupt local odometry even when place
recognition finds the correct region. Initialize while stationary; do not loosen
temporal agreement thresholds to hide this failure. Repeated structures and
inconsistent revisits can also yield alternative map fits. Evaluate initial-pose
correctness separately from NDT convergence, tracking coverage and clean exit.

## 5. DDS and bounded input memory

Both presets use BEST_EFFORT. ROS callbacks enqueue sensor work; scan matching
runs on a separate ordered worker, so expensive estimation does not block the
subscription callback. A RELIABLE publisher can match a BEST_EFFORT subscriber;
the reverse reliability pairing cannot match. See
[ROS QoS compatibility](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html#qos-compatibilities).

| Layer | Limit / policy |
|---|---|
| DDS LiDAR history | KeepLast 16 |
| DDS IMU history | KeepLast 1000 |
| Application work queue | 4096 messages and 128 MiB estimated payload, plus one in flight |
| LIO synchronization | 32 paired clouds/timestamps, 2000 IMU samples |
| Fast DDS SHM | 64 MiB per participant; max message 8 MiB; 4096 descriptors |
| Fast DDS UDP | Requested send/receive buffers 4 MiB each; OS may limit them |

Queue weight estimates include raw cloud storage and a small header allowance.
The worker pops one item at a time; it does not retain an entire drained batch
while new work accumulates. Overflow drops the oldest queued messages and logs
`queue overflow`; shutdown drains queued work. These are explicit memory bounds,
not guarantees against data loss or a total process cap. Loop-closure keyframes
are retained rather than evicted as arbitrary sensor messages.

M20 serialized scans reach about 4,096,835 bytes in the reference recordings.
Fast DDS 2.6's default 512 KiB SHM segment is too small; the enlarged segment
fixed the severe local transport loss seen with large clouds. Segment size is
per participant, not per scan. Apply it to publisher and subscriber, restart
existing processes, and retain the driver's own workspace. See
[Fast DDS SHM sizing](https://fast-dds.docs.eprosima.com/en/2.6.x/fastdds/transport/shared_memory/shared_memory.html).
Local SHM tests establish neither reliable best-effort delivery nor cross-host
UDP/physical-driver performance.

## 6. Build, runtime resources, and viewer lifecycle

`scripts/build.sh` extracts Pangolin from the bundled zip, disables examples,
tools, Python, and OpenEXR support, installs to `.deps`, and uses colcon to build
the package. The scripts accept Foxy and Humble and select ROS apt packages using
`ROS_DISTRO`. CMake records the build's distribution in the installed package;
`setup.bash` loads its colcon underlay, the local install, and Pangolin libraries,
and rejects a conflicting already-sourced ROS distribution.
It defaults to Fast DDS/the supplied profile, `OMP_NUM_THREADS=4`,
`OMP_WAIT_POLICY=PASSIVE`, `OPENBLAS_NUM_THREADS=1`, and `PANGOLIN_WINDOW_URI=x11://`.
Explicit environment overrides are preserved.

Every application creates a four-way `tbb::global_control` limit for parallel
point processing. NDT separately requests four OpenMP workers. ROS, loop closure,
and visualization have additional threads, so four is not the process's total
thread count. Passive OpenMP waiting reduces idle worker contention. See
[OpenMP waiting policy](https://www.openmp.org/spec-html/5.0/openmpse55.html) and
[oneTBB global control](https://uxlfoundation.github.io/oneTBB/main/specification/source/task_scheduler/scheduling_controls/global_control_cls.html).

The x86 tests used software OpenGL; display-server memory and graphics-thread
cost are not included in application RSS. Start native RK3588 validation with
`scripts/build_robot.sh`, normal sensor rate, and a representative long route.
Disable the viewer for headless operation. Lower map-match frequency if necessary, then check
tracking quality. Changing sampling/voxels requires reconstruction regression
checks. Increasing queues cannot fix sustained compute overload.

Pangolin initializes its viewport before rendering. Keyframe display clouds are
cached/sampled; OpenGL buffers are destroyed on the render thread. Verify actual
map/trajectory pixels, camera rotation, zoom, and Follow. A blank nested X server
window is not a valid visualization test. Offline processing closes the viewer
after export/completion; online shutdown follows Ctrl+C after explicit saving.

The optional `--rviz` flag on either online application creates the publisher in
`src/wrapper/online_visualization.cc`. It publishes a deskewed current scan and
its matching LiDAR pose at up to 5 Hz, plus a retained session path at 1 Hz, without
publishing map clouds. `config/onboard.rviz` selects those three displays. The
existing localization pose/TF output and all estimator settings are unchanged.
The public onboard sections describe SSH X11 forwarding; detailed maintenance
constraints are in `AGENTS.md`.

## 7. Map-save contract

Point-cloud filtering shares one helper. Ordinary clouds retain PCL's filtering
path; clouds whose grid would overflow PCL's signed 32-bit indices use sorted
sparse 64-bit coordinate tuples and the same centroid calculation instead.
This preserves leaf size while keeping memory proportional to input points,
including fine loop-closure submaps and large map exports. Non-finite points are
reported and excluded; invalid leaf sizes or coordinates outside the supported
64-bit range fail explicitly. No filtering fallback changes estimator poses,
calibration, or the requested spatial resolution.

Offline mapping saves to `--map_path` at normal bag completion. Online mapping
uses `/lightning/save_map` (`lightning/srv/SaveMap`) with `map_id`, relative to the
mapping node's `data/` directory. Wait for the service response before stopping
the node; queued loop closures must finish before export. A nonempty destination
is never silently overwritten.

| Save response | Meaning |
|---|---|
| 0 | Success |
| 2 | Invalid map ID or path exception |
| 3 | No map data, nonempty destination, or export failure |

The full directory comprises `global.pcd`, `index.txt` (tile index and starting
pose), numbered PCD tiles, and `places.bin` for global initialization. The onboard
launcher also saves `map_view.json` when it captures a stable startup gravity
reference. Copy/rename the complete directory for deployment, retaining both
sidecars when present. Older maps can omit them: ordinary/manual localization
uses the tiles, and saved-map viewing retains the original frame. Localization
cannot load a standalone global PCD as a complete tiled map.
