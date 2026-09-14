# Agent guidance

## 1. Scope and user intent

This repository publishes Lightning-LM mapping and localization for M20 Pro and
Livox Mid360. Keep exactly one maintained sensor preset for each:
`config/m20_pro.yaml` and `config/mid360.yaml`. All seven reference recordings
must work with these presets; avoid route-specific configuration forks.

Read [README.md](README.md), [implementation notes](doc/implementation.md), and
[validation records](doc/validation.md) before changing runtime behavior. This
file applies throughout the repository. User instructions take precedence.
Continue authorized, reversible work without adding approval checkpoints.

The public README is for people running the software. Keep its sections numbered,
commands easy to copy, and explanations, technical details, and results inside
closed-by-default `<details>` subsections. Keep development dates, branch names,
historical experiments, machine-specific paths, and long logs out of the README.
Maintain the text-free seven-panel header image; never synthesize, warp, or
selectively clean point clouds to improve the reported result.

Describe the Livox preset as Mid360 support, not as support for the robot that
carried the reference sensor. Historical evidence still names `lite3_edu.yaml`;
that file was renamed to `mid360.yaml` with identical parsed YAML settings.
Legacy dataset directory names are actual external paths and must not be renamed
just to match product wording. Do not retain a duplicate legacy production preset.

## 2. Repository boundaries

Preserve unrelated user changes. Inspect `git status` before editing: this
workspace has a substantial publication cleanup in progress, including staged
vendor deletions and unstaged changes. Do not reset, switch branches, or restore
deleted experiments over the user's work. Use isolated worktrees for historical
comparisons and record which binary/configuration actually ran.

Production entry points are the four `src/app/run_{slam,loc}_{offline,online}.cc`
programs. The supported scripts are `scripts/install_dep.sh`, `scripts/build.sh`,
`scripts/build_robot.sh`, and `scripts/setup.bash`. Keep experimental runners,
screenshot/render scripts, profiling helpers, temporary test programs, and unrelated configs out of the
published tree. Keep new investigation reports, verification records, images,
logs, maps, and binaries in ignored `outputs/` or external directories, and
report results to the user. Do not add session-specific validation artifacts to
the public tree. Preserve historical evidence already in `doc/`. Keep durable
maintenance guidance here and user-facing operating instructions in the README.
Do not delete existing evidence during cleanup.

Build products live under `build*`, `install*`, `bin`, `log*`, and `.deps`.
`thirdparty/Pangolin-0.9.3.zip` is the source dependency; its extracted directory
is ignored. Keep required Sophus, Miao, and Livox message definitions and licenses.
The repository's `livox_ros_driver2` package contains messages only. It is not a
hardware driver or a replacement for the driver's launch workspace.

## 3. Build and platform contract

Run from the repository root in a clean Bash environment:

```bash
source /opt/ros/humble/setup.bash
bash scripts/install_dep.sh
bash scripts/build.sh
source scripts/setup.bash
ros2 pkg executables lightning
ros2 interface show lightning/srv/SaveMap
ros2 interface show livox_ros_driver2/msg/CustomMsg
```

Use all workstation CPUs unless the user specifies otherwise. On the 16 GB
RK3588 target use `scripts/build_robot.sh`: it selects up to four compiler jobs,
reducing this according to currently available RAM. Runtime threading is
controlled separately. Do not enable x86 flags on ARM or make `-march=native`
a portable binary default.

For build comparisons, use the same job count and reduce it when measured compiler
memory would exceed available RAM. Report fresh builds, incremental builds, and
cache hits separately; compiler profiling with `-ftime-report` adds overhead and
is not a substitute for timing the ordinary build. Keep benchmark scripts and
measurements in ignored outputs.

RK3588 firmware can isolate A76 CPUs from scheduler load balancing. An affinity
mask covering several isolated CPUs can still leave both compiler jobs on one
core. Check actual CPU placement when reporting onboard build performance;
`CMAKE_BUILD_PARALLEL_LEVEL=2` means two jobs, not necessarily two occupied cores.
Preserve firmware real-time scheduling and sensor/control services.

The robot wrapper delegates dependency handling to `install_dep.sh` and compilation
to `build.sh`. Its default and `--check` modes never install/download packages;
`--offline DIRECTORY` installs only transferred packages before building. Keep the
laptop source transfer and AOS compilation visible in their own README
section, including downloaded ZIPs without Git metadata. The public workflow uses
the standard AOS dependencies; do not add a separate dependency-download tutorial.
Audit the target first and bundle any actually missing packages with the sources
for a deployment that needs them. The source archive must
contain the bundled Pangolin ZIP, Sophus, Miao, and message/service definitions,
and exclude laptop build products.

`build_robot.sh` detects RK3588 through device-tree compatibility and leases CPUs
4–7 to individual compiler children with `flock`/`taskset` for both Pangolin and
Lightning. Keep both phases on these four A76 cores; the user explicitly declined
using the smaller A55 cores for additional jobs.
Prefer A76 cores in order 7, 6, 4, 5 for the standard firmware's control/sensor
placement, so trailing compiler jobs use the less occupied cores first.
Other Linux systems use their inherited CPU affinity. `ROBOT_BUILD_CPUS` can explicitly select CPUs;
validate each requested CPU before building. Never change firmware isolation,
governors, real-time priorities, or service affinity. Build children use nice +10.
Preserve explicit C/C++ launchers, including an empty cache launcher, behind the
affinity wrapper. Keep lock files in ignored `build-robot-locks`; CMake may invoke
the recorded launcher again after the wrapper exits. Forward compiler failures.

The initial memory budget reserves 2 GiB of `MemAvailable` and allows 2.75 GiB
per Release compiler (4.5 GiB for other build types with debug symbols).
Default to at most four jobs; reject an explicit job count above
the memory/CPU allowance. This is not a runtime RAM cap: changing compiler flags,
future translation units, or other running software can increase demand.
Measure both aggregate process-tree RSS (shared pages may count twice) and minimum
system `MemAvailable` when changing the budget. A single child's maximum RSS is
not total build memory. Compare cache-disabled fresh builds with identical
sources/options and verify actual compiler placement on isolated cores.
Extract Pangolin with `unzip -DD -nq`: use local timestamps for newly extracted
files and leave existing files alone. Future archive timestamps otherwise trigger
recompilation under a robot's synchronized clock. Never fix this by touching
existing build products or changing the robot's clock synchronization.
Future-dated system shared libraries can still trigger relinking even when no
C/C++ source recompiles. Diagnose those dependencies separately; do not change
system-library timestamps or disable dependency tracking to improve a benchmark.

C++ Release builds retain `-O2` and enabled assertions; do not silently replace
them with CMake's `-O3 -DNDEBUG` defaults. Debug symbols are selected through
`CMAKE_BUILD_TYPE=RelWithDebInfo` or `Debug`, including when using `scripts/build.sh`.
The script detects ccache for both Pangolin and Lightning-LM, respects explicit
`CMAKE_C_COMPILER_LAUNCHER` / `CMAKE_CXX_COMPILER_LAUNCHER` environment settings,
and clears stale launchers when no cache is available. Keep normal cache validity
checks enabled.

For offline deployments, archive the selected sources with all required vendored
files; do not transfer x86 build products to ARM. `install_dep.sh --check` only
reports missing required packages; ccache is optional for this check because
`build.sh` can compile without it. Online installation and explicit offline
package requests still include ccache. `--download-uris` resolves dependency downloads
against the target's installed state and trusted APT indexes, using an empty
cache so already-cached packages are included in the request. Run it on AOS,
then download those exact URIs on the connected computer with APT's requested
filenames. `--offline DIRECTORY` must retain `--no-download` and `--no-remove`,
use the transferred cache, and never run `apt-get update`. APT verifies cached
packages against its indexes. Missing/stale indexes need an offline index update;
do not substitute the workstation's architecture, distribution, or package state.
Keep the online installer's package list as the single dependency list for all
modes. Check both Foxy and Humble argument paths and preserve installation errors.

Keep `common/so3_math.hpp` independent of PCL and ROS. The non-template PCL voxel
filter and SVD helper belong in `core/lightning_math.cc`, so including the math
header does not instantiate these algorithms in every consumer. Include Eigen
decomposition headers and other dependencies where they are used. Avoid pulling
IMU processing or NDT implementation headers into public module headers merely
to declare pointers.

The project supports Ubuntu 20.04 / Foxy and Ubuntu 22.04 / Humble. The user has
verified both versions and explicitly requested this support statement. This
session's detailed seven-dataset/build evidence comes from Humble on x86-64;
keep its provenance distinct from the user's platform verification. Do not label
Foxy unsupported or unverified in the README. Preserve compatibility with both
distributions when changing generated interfaces/typesupport, rosbag2 APIs,
TBB/PCL/OpenCV dependencies, and Fast DDS XML. Stock Foxy's bag player lacks
`--clock` and `--delay`; the README uses the shared `--rate` option. ROS `/clock`
is optional for this algorithm, which uses acquisition timestamps directly.

Build from publishable sources without existing build/install directories when
changing installation or dependency handling. Use a new export/worktree rather
than destroying the user's only build. Confirm `ros2 run` resolves this checkout,
and that its installed libraries do not depend on another checkout's `.deps`.
When renaming installed configs, remove only the obsolete generated copy after a
successful install; CMake's directory install does not prune old filenames.

## 4. Algorithm invariants

- Preserve full 3D estimation. `loop_closing.with_height`, `lidar_loc.force_2d`,
  and `system.with_g2p5` stay false in both presets. Never use a z constraint to
  disguise drift; uneven terrain and multi-level structures must remain possible.
- M20 uses inertial translation and gyro rotation. Mid360 uses constant-velocity
  translation and gyro rotation. In the latter, the mean and Jacobian must agree:
  do not reintroduce acceleration/bias/gravity coupling into translation while
  its mean acceleration is disabled. Retain velocity uncertainty and LiDAR updates.
- Keep point timestamps through native preprocessing and deskewing in both
  mapping and localization. LiDAR headers, per-point times, and IMU headers need
  a consistent time base, not identical arrival times. There is no automatic
  LiDAR–IMU clock-offset calibration. `--clock` does not synchronize sensors.
- Preserve matching cloud/timestamp queue entries when rejecting stale input.
  M20 bags contain out-of-order LiDAR headers; received counts and processed
  odometry scans are different metrics.
- Treat extrinsics as calibration. `p_imu = R * p_lidar + T`; Mid360's `[0,0,0.28]`
  is the tested mounting, not a universal sensor value. Do not tune it to square
  walls or flatten a map.
- Reject non-finite, unconverged, or insufficient-confidence localization
  matches. Do not restore unconditional success. Keep map targets updated after
  loading tiles, export only valid matches, and preserve reference maps.
- Preserve loop rejection and graph bookkeeping. An edge marked inactive must
  cease contributing to the solve. Do not assume Miao's fixed-vertex interface
  implements every incremental-solver operation safely without checking it.

See [implementation notes](doc/implementation.md) for files, equations, effective
settings, timestamp units, memory limits, and lifecycle behavior.

## 5. DDS, resources, and deployment

Both sensor presets MUST retain best-effort subscriptions: the robot publishes
best effort. Reliable publishers also match these subscriptions, so a Mid360
driver does not need a reliability change solely for Lightning-LM. Larger SHM
buffers are a transport fix, not a reason to require reliable delivery.

The supplied Fast DDS profile uses a 64 MiB segment per participant, 8 MiB maximum
message, and 4096 descriptors. Source setup in application/player terminals.
Export the middleware/profile in the separately sourced physical driver
workspace and restart its publisher. Do not shadow the real Livox driver with
this repository's message-only package. SHM evidence applies to local processes;
cross-host UDP and new physical-driver deployments still require validation.

### M20 Pro onboard operation

- NOS is `10.21.31.106`; `multicast-relay.service` permits point-cloud access.
  Start it before testing LiDAR delivery; enabling it at boot is optional.
- AOS normally uses `10.21.33.103`. The additional network adapter used in the
  hardware session exposes it at `10.21.41.1`; do not present that address as the
  default for every Wi-Fi connection.
  Use a root application shell and source `/opt/robot/scripts/setup_ros2.sh`,
  then explicitly select this checkout's Fast DDS XML before sourcing setup.
  An ordinary-user probe received IMU but no LiDAR; root received both with the
  firmware and checkout profiles. Discovery alone does not prove delivery.
- Keep separate, complete onboard mapping and localization workflows in the
  README, alongside the recorded-data workflows. Both use the same online
  executables, sensor presets, and tiled-map format. The headless runtime copy
  changes only `system.with_ui`; do not add separate production SLAM/loc presets.
  When consulting another branch's hardware guide, verify commands against this
  branch's executables, services, and topics before documenting them.
- Keep firmware drivers and control services running. The firmware already
  publishes `map` to `base_link` on `/tf`. For independent Lightning-LM use,
  remap its TF and initial-pose topics as shown in the README. Gflags requires
  the `--` separator before `--ros-args`; verify resolved endpoints.
- Record application CPU affinity separately from build parallelism. An AOS root
  SSH shell can inherit only CPUs 0–3 (A55). An explicit A76 application launch
  can help diagnose sensor backlog, but compiler placement is not a runtime
  performance qualification. Preserve failed queue-overflow trials and check
  normal-rate delivery when testing application affinity.
- AOS may be synchronized by `ptp4l`/`phc2sys`. A manually set wall clock can be
  immediately replaced by the robot's clock source. Do not disable sensor
  synchronization to remove build timestamp warnings. Normalize timestamps only
  in a new source export when necessary; do not touch all existing build products
  during incremental-build measurements.
- The stationary run used an external copy of `m20_pro.yaml` with only
  `system.with_ui: false`. It exercised online mapping, save-service export,
  localization, and separate saved-map rendering. A one-keyframe stationary map
  does not qualify moving routes, loop closure, larger maps, or onboard rendering.
- Preserve Foxy/PCL 1.10 and Humble/PCL 1.12 compatibility on x86 and ARM:
  ROS service callbacks take request shared pointers by value for Foxy's exact
  callback traits. Construct clouds through the PCL `CloudPtr` alias; older PCL
  uses Boost pointers and newer PCL uses standard-library pointers. Never replace
  this alias with an unconditional `std::make_shared` for PCL clouds.

Keep bounded sensor queues, four-worker NDT/oneTBB limits, passive OpenMP waiting,
5 Hz map localization, and distant-tile unloading unless measurements justify a
change. All received native scans and IMU samples still enter odometry. Queue
sizes are not a total memory cap, and SLAM keyframe memory grows with route length.
RK3588 has 16 GB shared with the OS and other robot software. Workstation RSS is
not an onboard performance guarantee. Avoid heavy concurrent GUI runs when
diagnosing real-time delivery. Never call best-effort delivery lossless.

## 6. Verification proportional to the change

For documentation/naming changes, check links, default-closed details, CLI help,
installed configs, parsed YAML equivalence, and a representative replay when a
runtime path or command changes. Do not rerun every long recording solely for
prose edits or add implementation-mirroring tests.

For estimator, filtering, timing, transport, concurrency, map I/O, or localization
changes, repeat the affected checks across all seven bags and both families.
Use [the validation contract](doc/validation.md). A full functional qualification
includes four workflows per recording (28), visualization, map saving, valid
trajectories, and normal-rate online playback. Check default reliable bag
publishers as well as explicitly best-effort publishers. A clean exit or plausible
screenshot alone is insufficient.

Use unique map/output directories. Online mapping saves through
`/lightning/save_map`; wait for success before stopping it. Localization needs
`index.txt` plus all tiles, not just `global.pcd`. Verify a loaded reference map's
hashes remain unchanged. Inspect actual rendered pixels and readable PCD data,
not merely window existence and PCD headers. Keep the gallery and live viewer
checks separate. A nested X server's black desktop is not a rendered SLAM view.
The mapping section ends with `pcl_viewer` commands for saved offline/online
clouds; keep `pcl-tools` in the installer and verify those commands when changing
PCD export or visualization dependencies.
Do not leave test windows/servers/replays running after finishing, or terminate
unrelated user sessions while cleaning up.

Preserve exact commands, input counts, configuration snapshots/hashes, build
identity, timing, resource metric definitions, map checks, and failed trials.
Record a new result when code changes; never rewrite historical evidence to
claim it was tested with a newer configuration. The rename provenance in
[validation.md](doc/validation.md) explains the unchanged reference settings.

## 7. Known limits and next deployment work

Library F remains the hardest recording and has visible revisit misalignment.
There is no surveyed ground truth. “Pass” means the documented operational
checks passed, not perfect geometry or a guaranteed successful future run.
One normal-rate best-effort Building 3 trial lost one IMU message without a queue
overflow; the unchanged full-input repeat does not invalidate that observation.

The remaining deployment work is profiling the current resource settings on the
16 GB RK3588, physical-driver timing/calibration validation for a new mounting,
and cross-host DDS testing if required. Keep platform support separate from
hardware performance measurements. See `doc/validation.md` for local bag paths, archives, rejected
experiments, and the distinction between historical proposals and completed work.

### Onboard RViz2 maintenance

- Both online applications accept `--rviz`. This opt-in output is independent of
  `system.with_ui`, which controls Pangolin. Keep the production sensor YAMLs
  unchanged; onboard instructions disable Pangolin in an external runtime copy.
- `wrapper/online_visualization.{h,cc}` accepts scans on the existing ordered
  sensor worker. A one-second wall timer in the ROS executor copies path state
  under a mutex and serializes it after releasing the lock; this also delivers
  the final path samples when sensor input stops. Publish only the current
  deskewed scan, matching LiDAR pose, session
  path, and `map` → `lightning_lidar` transform. Never publish map/keyframe clouds
  for this view. Mapping applies the latest keyframe correction and LiDAR
  extrinsics; localization uses accepted NDT scan poses. Do not use an IMU-time
  pose to place an earlier scan or publish rejected localization matches.
- Keep `/lightning/pose` and its existing localization TF behavior unchanged.
  RViz uses `/lightning/current_pose`, `/lightning/current_scan`,
  `/lightning/trajectory`, and the separate `/lightning/tf`. Use frame `map` for
  all three display messages; the dedicated TF child avoids firmware `base_link`
  conflicts and permits camera following. Remap RViz's TF subscriptions away
  from firmware topics.
- Limit pose/scan output to 5 Hz and publish the full retained path at 1 Hz with
  reliable, transient-local, depth-one QoS. Retain history before a viewer joins;
  reopening RViz must recover it. History starts over when the application
  restarts, grows with session duration, and preserves estimates as recorded
  rather than retrospectively optimizing the trail. Scan output is best effort,
  depth one, converted only with a subscriber, and RViz uses zero decay so only
  the latest scan is drawn. Display serialization never modifies estimator clouds.
- Install `config/onboard.rviz` with the other configuration assets and keep its
  five-frame-per-second view free of map displays. The new `.rviz` asset is not
  another sensor preset. Keep RViz2 and xauth in dependency checks for both ROS
  distributions; audit the offline robot image before adding download steps.
- Run RViz2 on AOS through standard SSH X11 forwarding; MobaXterm is an optional
  Windows client, not a software dependency. Preserve DISPLAY and the original
  user's XAUTHORITY when entering a root shell. Software Mesa, two rendering
  threads, and disabled MIT-SHM support forwarded displays. Do not expose an
  unauthenticated X server or replace DISPLAY with a laptop address. X11 drawing
  traffic still consumes Wi-Fi bandwidth even though ROS scan/map data stays on
  AOS. Measure the forwarded SSH stream when making bandwidth claims.
- On standard RK3588, test the application on A76 core 7 and RViz on A76 core 6
  together with live sensors. Do not change firmware affinity, control services,
  or sensor clock synchronization. Record input queues, process memory, actual
  rendered pixels, and display topic/frame/QoS behavior. A stationary hardware
  check must be paired with moving recorded data to verify a visible route.
  Measure acquisition cadence, callback arrival intervals, algorithm processing
  intervals, and actual RViz frame intervals separately. A configured frame-rate
  cap or a topic frequency does not establish smooth rendering over SSH.
  A blank nested X desktop is a test harness, never a successful RViz capture;
  close it between tests and clean up only processes started by the test.

### Jetson AGX with Livox internal IMU

- Reach the AGX through the robot with SSH ProxyJump and forward X11 directly
  from the AGX. The jump host needs no X server. Private IPs can overlap another
  laptop interface or a previous device; verify the target key using the trusted
  jump host's existing record, and use a distinct HostKeyAlias. Preserve old
  known-host records instead of blindly removing mismatches.
- Audit the AGX's actual ROS distribution, dependencies, RAM and driver workspace.
  Use the existing build scripts and the AGX's available CPUs. RK3588 CPU numbers
  and firmware-affinity assumptions do not apply to Jetson. Keep builds and test
  runs separate when measuring runtime throughput.
- Keep the full Livox hardware driver in its own sourced workspace. Match the
  launch/configuration to Mid360 versus Mid360s and check the installed config's
  host Ethernet address and sensor IP. A running ROS node alone does not prove
  packets are being received. Preserve pre-existing driver/configuration edits.
- An independent all-on-AGX pipeline can use ROS_DOMAIN_ID=42 and
  ROS_LOCALHOST_ONLY=1 in driver, application, service and RViz terminals. This
  isolates DDS from robot firmware while the hardware driver receives Ethernet
  packets. Start one owned driver instance and stop only owned test processes.
- For the internal Livox IMU, subscribe to /livox/imu with /livox/lidar. The
  Mid360 and Mid360s manuals define parallel axes and put the IMU origin at
  [0.011, 0.02329, -0.04412] metres in LiDAR coordinates. For this project's
  p_imu = R * p_lidar + T convention, use R=I and
  T=[-0.011, -0.02329, 0.04412]. Verify the driver's point transform is identity.
  Apply this calibration in an external runtime copy of config/mid360.yaml;
  preserve the historical recording preset and its evidence. A robot URDF's
  LiDAR-to-body transform is separate and must not replace this internal transform.
- Check sensor header cadence, per-point timing, acquisition spans and IMU
  coverage before diagnosing a clock-offset problem. Full Python CustomMsg
  decoding can saturate a probe on ARM and create measurement backlog. Use a
  lightweight raw-CDR or C++ observer; distinguish acquisition timestamps from
  callback arrival and processing time. Do not compensate probe backlog with
  estimator timestamp shifts or an adaptive offset filter.
- The existing Livox driver may substitute host receipt timestamps for unsynced
  packets. Record the actual driver version and timestamp path; coherent output
  under that mode is not hardware/PTP synchronization qualification.
- Forwarded Jetson RViz can use __GLX_VENDOR_LIBRARY_NAME=mesa together with
  LIBGL_ALWAYS_SOFTWARE=1, LP_NUM_THREADS=2 and QT_X11_NO_MITSHM=1. Inspect real
  scan/arrow/path pixels and frame timing after initialization. Preserve the
  existing display rates, QoS, full-session trajectory and absence of map topics.
