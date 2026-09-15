# Guidance for AI agents

## 1. Start here

This file guides AI assistants working anywhere in this repository. Read it before
editing. User instructions take precedence; continue authorized, reversible work
without adding approval checkpoints.

Lightning-LM supports **Deep Robotics M20 Pro** and **Lite3 EDU with Jetson AGX
and Livox Mid360**. Keep exactly two maintained sensor presets:
`config/m20_pro.yaml` and `config/mid360.yaml`. The Mid360 preset describes the
sensor and recorded mounting; live Lite3 EDU setup creates a local runtime copy
for the LiDAR's internal IMU. Do not create per-route or per-workflow presets.

Before working:

1. Inspect `git status` and preserve unrelated changes. Identify the requested
   outcome and the checks that would demonstrate it.
2. Read [README.md](README.md) for supported commands. Before changing runtime
   behavior, also read [implementation.md](doc/implementation.md) and
   [validation.md](doc/validation.md).
3. Investigate the cause before patching symptoms. Make the smallest coherent
   change and run the relevant checks in section 7.
4. Report what changed, the exact build/configuration tested, measured results,
   failures, and remaining limits. Keep new test artifacts out of the public tree.

The historical `lite3_edu.yaml` preset was renamed to `mid360.yaml` with identical
parsed settings. Preserve historical evidence and actual dataset directory names;
do not restore a duplicate legacy preset or reinterpret the rename as a new test.

## 2. Repository and documentation boundaries

| Purpose | Maintained entry points |
|---|---|
| Mapping and localization | `src/app/run_{slam,loc}_{offline,online}.cc` |
| Saved-map display | `src/app/publish_map.cc` (read-only PCD publisher) |
| RViz lifecycle | `src/app/run_rviz.cc` (installed RViz components) |
| Dependencies and build | `scripts/install_dep.sh`, `scripts/build.sh`, `scripts/build_robot.sh` |
| Runtime setup | `scripts/setup.bash`, `scripts/onboard.py` |

Do not reset, switch branches, or restore deleted files over user work. Use an
isolated export or worktree for historical comparisons and clean-build checks.
Record which sources, binary and configuration actually ran.

Keep new investigation scripts, reports, screenshots, logs, maps, test programs
and binaries in ignored `outputs/` or outside the repository. Preserve existing
historical evidence in `doc/`; do not rewrite it to claim a later build was tested.
Build products belong under `build*`, `install*`, `bin`, `log*`, and `.deps`.
Keep the bundled `thirdparty/Pangolin-0.9.3.zip`, Sophus, Miao, message/service
sources and their licenses. The extracted Pangolin directory is ignored.
The bundled `livox_ros_driver2` contains messages only, not a hardware driver.

Keep the README useful to operators: numbered sections, copyable commands and
closed-by-default `<details>` for extended explanations, technical details and
results. Keep dates, branch names, machine-specific paths, experiments and long
logs out of it. Put durable maintenance guidance here, implementation explanations
in `doc/implementation.md`, and new session evidence in ignored outputs.

Preserve the README structure:

- Section 5: laptop source transfer, compilation and installation, with separate
  M20 Pro/AOS and Lite3 EDU/AGX subsections.
- Section 6: connections and setup, including one combined Lite3 EDU/AGX/Mid360
  subsection for the driver, calibration and saved environment.
- Sections 7 and 8: shared mapping and localization commands using `onboard.py`.

Maintain the text-free seven-panel header image. Never synthesize, warp or
selectively clean clouds to improve a reported reconstruction.

## 3. Build and platform contract

### Supported environments and entry points

Preserve Ubuntu 20.04 / ROS 2 Foxy and Ubuntu 22.04 / ROS 2 Humble compatibility
on x86-64 and ARM. The user's verification of both ROS versions is distinct from
the detailed seven-recording Humble/x86 evidence. Do not label Foxy unsupported.

Run from the repository root in a fresh Bash environment:

```bash
source /opt/ros/humble/setup.bash
bash scripts/install_dep.sh
bash scripts/build.sh
source scripts/setup.bash
ros2 pkg executables lightning
ros2 interface show lightning/srv/SaveMap
ros2 interface show livox_ros_driver2/msg/CustomMsg
```

Use `/opt/ros/foxy/setup.bash` on Ubuntu 20.04. Keep generated interfaces,
typesupport, rosbag2, TBB/PCL/OpenCV and Fast DDS XML compatible with both versions.
Foxy service callbacks take request shared pointers by value. Construct PCL clouds
through the `CloudPtr` alias: PCL 1.10 uses Boost pointers, while PCL 1.12 uses
standard-library pointers. Do not unconditionally substitute `std::make_shared`.
Stock Foxy's bag player lacks `--clock` and `--delay`; shared instructions use
`--rate`. The estimator uses acquisition timestamps and does not require `/clock`.

When changing installation or dependencies, build a fresh source export without
existing build/install directories. Confirm `ros2 run` resolves that checkout and
installed libraries do not depend on another checkout's `.deps`. After renaming
installed configs, remove only the obsolete generated copy after successful
installation; CMake's directory install does not prune old names.

### Compilation, caching and resources

Use all workstation CPUs when RAM permits. Reduce parallelism when measured
compiler demand would exceed available memory. On robots use `build_robot.sh`;
its at-most-four-job build budget is separate from runtime threading. Audit AGX
ROS, dependencies, RAM, available CPUs and the full driver workspace; RK3588 CPU
numbers and isolation assumptions do not apply to Jetson.

- Preserve Release `-O2` with assertions enabled, not CMake's `-O3 -DNDEBUG`.
  `Debug` and `RelWithDebInfo` select debug symbols. Do not make `-march=native`
  a portable default or apply x86 flags on ARM.
- Keep ccache detection for both Pangolin and Lightning. Respect explicit C/C++
  compiler launchers, including an explicitly empty launcher; clear stale cache
  launchers when no cache is available. Keep normal cache-validity checks.
- Compare fresh builds, incremental builds and cache hits separately with the
  same sources/options/job count. `-ftime-report` adds overhead. Measure aggregate
  process-tree RSS and minimum system `MemAvailable`; a single child's maximum
  RSS is not total build memory, and shared pages may count more than once.
- Keep `common/so3_math.hpp` independent of PCL and ROS. Non-template voxel/SVD
  helpers belong in `core/lightning_math.cc`. Include decomposition dependencies
  where used; do not pull IMU/NDT implementations into headers just for pointers.
- Extract Pangolin with `unzip -DD -nq`, preserving existing files and using local
  timestamps for new files. Future archive timestamps can force recompilation.
  Diagnose future-dated system libraries separately. Do not touch existing build
  products/system libraries, disable dependency tracking, or change robot clock
  synchronization to improve a benchmark.

On 16 GB RK3588, `build_robot.sh` detects device-tree compatibility and leases
individual compiler children to A76 CPUs 4–7 with `flock`/`taskset`, for both
Pangolin and Lightning. Prefer cores 7, 6, 4, 5 in that order. The user declined
additional A55 jobs. An affinity mask over isolated CPUs does not ensure jobs
occupy different cores: check actual placement. Never alter firmware isolation,
governors, real-time priorities, or service affinity. Build children use nice +10.

Other platforms retain inherited affinity. Validate every `ROBOT_BUILD_CPUS`
entry. Preserve explicit launchers behind the affinity wrapper, forward compiler
failures, and retain ignored `build-robot-locks` because CMake may invoke the
recorded launcher later. The initial robot budget reserves 2 GiB of `MemAvailable`
and allows 2.75 GiB per Release job (4.5 GiB with debug symbols), at most four jobs.
Reject explicit counts exceeding CPU/RAM allowances. This is not a runtime RAM
cap; remeasure when compiler flags, translation units or competing loads change.
Keep compilation separate from runtime throughput measurements.

### Offline deployment and dependencies

The robot wrapper delegates dependencies to `install_dep.sh` and compilation to
`build.sh`. Default and `--check` modes never install/download packages.
`--offline DIRECTORY` installs only transferred packages before building.
Use the standard robot environment; audit missing dependencies before adding any
download steps. Bundle actually missing packages with the sources, rather than
adding a separate dependency-download tutorial to the README.

Source transfers must work from Git checkouts or downloaded ZIPs without Git
metadata. Include Pangolin ZIP, Sophus, Miao and message/service definitions;
exclude build products, `__pycache__` and `.pyc`. Never transfer x86 binaries to ARM.

- Keep one installer package list for all modes. `--check` treats ccache as
  optional; online installation and explicit offline requests include it.
- Resolve `--download-uris` on the target against its installed state and trusted
  APT indexes, with an empty cache so cached packages are still listed. Download
  those exact URIs and filenames on the connected computer.
- `--offline DIRECTORY` must use `--no-download`, `--no-remove` and the transferred
  cache, never `apt-get update`. APT verifies packages against target indexes.
  Missing/stale indexes need an offline update; do not substitute workstation
  architecture, distribution or package state. Check Foxy/Humble argument paths
  and preserve installation failures.

## 4. Algorithm invariants

- Preserve full 3D estimation: `loop_closing.with_height`, `lidar_loc.force_2d`
  and `system.with_g2p5` stay false. Do not disguise drift with a z constraint.
- M20 uses inertial translation and gyro rotation. Mid360 uses constant-velocity
  translation and gyro rotation. Its mean and Jacobian must agree: no acceleration,
  bias or gravity coupling into translation when mean acceleration is disabled.
  Retain velocity uncertainty and LiDAR updates.
- Preserve native point timestamps and deskewing in mapping and localization.
  LiDAR headers, per-point times and IMU headers need a consistent time base,
  not equal arrival times. There is no automatic LiDAR–IMU clock-offset calibration.
  Preserve cloud/timestamp queue pairs when rejecting stale input; M20 recordings
  contain backwards headers, so received and processed scan counts differ.
- Extrinsics are calibration: `p_imu = R * p_lidar + T`. Mid360's `[0,0,0.28]`
  is the recorded mounting, not a universal sensor value. Never tune extrinsics
  to flatten a map or square walls.
- For the Livox internal IMU, use `/livox/imu` and `/livox/lidar`. Manufacturer
  Mid360/Mid360s geometry places the IMU origin at `[0.011,0.02329,-0.04412]` metres
  in LiDAR coordinates with parallel axes. This project's convention therefore
  uses `R=I`, `T=[-0.011,-0.02329,0.04412]`. Verify identity driver point transforms
  and apply this in an external runtime copy, preserving the recording preset.
  A robot URDF's LiDAR-to-body transform is a separate calibration.
- Reject non-finite, unconverged or low-confidence localization matches. Update
  map targets after tile loading, export only accepted poses, and preserve maps.
- Preserve loop rejection and graph bookkeeping. Inactive edges must stop
  contributing. Check Miao's fixed-vertex/incremental-solver behavior before
  assuming an operation is safe.

## 5. DDS and onboard deployment

### Shared transport and saved settings

Both sensor presets retain best-effort subscriptions; reliable publishers also
match. Larger SHM buffers solve transport capacity, not reliability policy.
The supplied Fast DDS profile uses a 64 MiB segment per participant, 8 MiB maximum
message and 4096 descriptors. Source setup in application/player terminals, and
export the same middleware/profile in the separately sourced physical-driver
workspace before restarting its publisher. Never shadow the hardware driver
with this repository's message-only package. Local SHM evidence does not qualify
cross-host UDP. Topic discovery alone does not prove delivery.

Configure `onboard.py` once with `configure m20` or `configure agx`. It loads
ignored `data/onboard.json` and `data/onboard.yaml` for every child command.
Repeated configuration preserves runtime YAML unless `--config` replaces it.
Do not persist DISPLAY/XAUTHORITY or edit shell startup files: X11 belongs to the
current connection. Build child environments from the chosen workspace, clearing
inherited ROS overlay paths while preserving SSH authorization. Never source
Lightning's message-only overlay into the full driver command.

Route sourced setup notices to stderr before launching a child that emits JSON.
M20 firmware setup prints network notices on stdout. Preserve its environment
and exit failures in the same shell; do not accept arbitrary trailing JSON or
edit firmware DDS files to hide the parsing error. Check noisy/failing setup and
actual delivery on Foxy/M20 and Humble/AGX. Configuration affects launcher children,
not the parent shell; direct `ros2` commands still need that shell's ROS/DDS setup.

`onboard.py status` is an operator-facing delivery check: report each sensor's
topic and received count, distinguish no publisher from a discovered publisher
with no delivery, and print SUCCESS or NOT READY with a relevant next step.
Mention an existing estimator instead of implying a duplicate can be started.
Preserve `status --json` for automation, the internal probe's JSON-only stdout,
and exit codes 0 (both streams arrive) / 1 (missing input or check failure).
Readiness is a short transport check, not calibration, clock or sustained-rate
qualification. Test missing LiDAR, missing IMU, both missing, discovered but silent
publishers, live success, and malformed/missing setup without stopping firmware.

### SSH disconnects and tmux

Keep the headless driver and algorithm inside tmux on AOS/AGX when sessions must
survive Wi-Fi loss. M20 sessions use root; AGX sessions use the driver/application
account. Sessions are scoped to a host and user. Reattach existing sessions after
reconnection; do not start duplicate estimators or assume tmux survives a reboot.
An already-running driver outside tmux is not adopted by `onboard.py lidar`.

Run RViz outside tmux in a fresh authenticated `ssh -Y -C` connection. An existing
X11 client cannot migrate when the original tunnel ends. Reopen the viewer with
the new connection's DISPLAY/XAUTHORITY, preserving authorization through M20
root elevation. The retained path recovers while the same estimator remains
alive. Do not restart estimation, persist display credentials, or add automatic
shell hooks to reopen viewers. Saved maps reopen with the existing `rviz --map`
command and require no estimator.

Document create/list/attach/detach/stop operations. Save mapping successfully
before Ctrl+C; wait for the shell prompt before exiting or removing a session.
Stop only the named test sessions and their owned applications, never a shared
tmux server or firmware services. Validate SSH-client loss and reattachment with
unchanged process identity, continuing sensor/pose delivery, retained path start
timestamps, actual reopened RViz pixels, and clean shutdown. Keep driver ownership
and stationary-test limits explicit.

When connecting robot Wi-Fi alongside laptop Ethernet, inspect routes, DNS and
link/driver logs before changing settings. Robot-only Wi-Fi profiles should not
replace the internet default route or DNS. Preserve unrelated profiles and VPN
settings; distinguish a reported carrier drop from proven physical unplugging.
Record local network changes and rollback information outside the public tree.

### M20 Pro / AOS

- NOS is `10.21.31.106`. Start `multicast-relay.service` for point-cloud access;
  enabling it at boot is optional. Preserve firmware sensor/control services.
- AOS normally uses `10.21.33.103`; `10.21.41.1` is an alternative adapter address,
  not a universal Wi-Fi default. Use a root application shell, source
  `/opt/robot/scripts/setup_ros2.sh`, then select this checkout's Fast DDS XML
  before sourcing setup. Ordinary-user probes can receive IMU without LiDAR.
- The saved profile selects estimation CPU 7 and RViz CPU 6. Root SSH can inherit
  only A55 CPUs 0–3. Record runtime affinity separately from compiler parallelism;
  preserve failed overflow trials and check normal-rate input delivery.
- Firmware already publishes `map` → `base_link` on `/tf`. Remap Lightning TF and
  initial pose as in the README. Gflags needs `--` before `--ros-args`; verify
  resolved endpoints. Leave firmware transforms and services running.
- AOS may use `ptp4l`/`phc2sys`; manually set wall time can be replaced immediately.
  Preserve synchronization. Normalize timestamps only in a new source export
  when necessary, not in existing builds used for incremental measurements.

### Lite3 EDU / AGX / Mid360

- Connect with SSH ProxyJump and forward X11 directly from AGX. The jump host
  needs no X server. Verify host keys against trusted records when private IPs
  overlap; use a distinct HostKeyAlias rather than removing mismatches blindly.
- Keep the full Livox driver in its own workspace. Discover a unique built
  workspace under the user's home or require an explicit path if ambiguous.
  Match Mid360 versus Mid360s launch/configuration, Ethernet host and sensor IPs.
  Preserve pre-existing driver/configuration edits and check actual packets.
- Live mapping/localization require both LiDAR and internal-IMU streams. Start
  one driver before the algorithm and keep it across runs. Restart only after
  it stops or AGX reboots; refuse to replace a discovered but non-delivering
  publisher automatically. Saved-map viewing needs no driver or estimator.
- A fully local AGX pipeline can use domain 42 and `ROS_LOCALHOST_ONLY=1` in
  driver/application/service/viewer environments, while the driver receives
  Ethernet packets. Retain inherited CPU affinity and software Mesa settings.

Keep bounded sensor queues, four-worker NDT/oneTBB limits, passive OpenMP waiting,
5 Hz map matching and distant-tile unloading unless measurements justify change.
All received native scans/IMU still enter odometry; queues are not a total RAM cap,
and mapping keyframe memory grows with route length. RK3588 shares 16 GB with the
OS and firmware. Workstation RSS does not guarantee onboard performance. Avoid
heavy concurrent GUIs when diagnosing delivery; never call best effort lossless.

## 6. Visualization maintenance

### Live scan, pose and trajectory

Both online applications accept opt-in `--rviz`, independent of Pangolin's
`system.with_ui`. Onboard runtime copies disable Pangolin; recording presets stay
unchanged. `wrapper/online_visualization.{h,cc}` accepts scans on the ordered sensor
worker. Its one-second wall timer copies path state under a mutex, then serializes
after unlocking, including final samples after sensor input stops.

Publish only the current deskewed scan, matching LiDAR pose, retained path and
`map` → `lightning_lidar` transform. Mapping applies latest keyframe corrections
and LiDAR extrinsics; localization uses accepted NDT scan poses. Never use an
IMU-time pose for an earlier scan, publish rejected matches, or modify estimator
clouds during serialization. Preserve existing `/lightning/pose` and localization
TF behavior.

RViz uses `/lightning/current_pose`, `/lightning/current_scan` and
`/lightning/trajectory` in frame `map`, with separate `/lightning/tf` subscriptions.
The child frame avoids firmware `base_link` conflicts and supports camera follow.
Limit pose/scan to 5 Hz. Scans are best effort, depth one, converted only with a
subscriber; zero decay shows only the latest scan. Full paths publish at 1 Hz,
reliable/transient-local/depth one, so late viewers recover history. History grows
with session length, resets on restart, and preserves recorded estimates rather
than retrospectively optimizing the trail. `config/onboard.rviz` stays at 5 FPS
without map displays. Install RViz assets, RViz2 and xauth for both ROS versions.

### Forwarded displays and saved maps

Use authenticated SSH X11 forwarding. MobaXterm is an optional Windows client;
Ubuntu needs no MobaXterm. Show complete laptop-origin `ssh -Y -C -J ...` commands
for additional AGX viewer terminals. Check SSH-assigned DISPLAY before Qt; ROS
exports cannot add X11 to an existing plain SSH session. Preserve the original
user's XAUTHORITY and DISPLAY through root elevation. Do not substitute `:0`, a
laptop address, an unauthenticated X server, or reinstall Qt for an empty DISPLAY.

Software Mesa uses `LIBGL_ALWAYS_SOFTWARE=1`, `LP_NUM_THREADS=2`,
`QT_X11_NO_MITSHM=1`; Jetson additionally selects `__GLX_VENDOR_LIBRARY_NAME=mesa`.
X11 drawing still uses Wi-Fi bandwidth even when ROS data stays onboard; measure
the SSH stream before making bandwidth claims.

`onboard.py rviz` loads the saved environment and checks DISPLAY.
`onboard.py rviz --map DIRECTORY` independently starts `publish_map` and
`saved_map.rviz`, without requesting live export or modifying keyframes.

- Use unique cloud/static-TF topics, reliable/transient-local/depth-one QoS, cloud
  frame `map`, and a camera fitted to full finite cloud bounds. A private parent
  `lightning_map_view` supports viewing without an estimator. Uniformly sample at
  most 500,000 XYZ points (under 8 MiB/message), report input/finite/displayed
  counts, and preserve source PCD coordinates/bytes. Sampling is display-only.
- Use `run_rviz` with installed RViz libraries/plugins. Stop its render timer
  before close events, nested save-dialog handling and shutdown; restart on
  canceled close. Forwarded Mesa can otherwise crash in XPutImage/OGRE teardown.
  Keep Qt5/RViz APIs compatible with Foxy/Humble; do not speculatively replace
  graphics libraries. Test close, canceled close, Ctrl+C and SSH hangup, including
  large maps and cleanup of both owned processes; report abnormal exits.
  A disconnected SSH PTY can raise EIO while reporting a viewer exit. Stop the
  saved-map publisher in its own `finally` so a cleanup/logging failure cannot
  strand it. Validate actual connection loss as well as sending SIGHUP alone.
- The launcher records a stable IMU mean while stationary before mapping.
  Estimation starts with identity rotation and gravity in initial IMU axes;
  map Z is not automatically vertical. Save the upward vector in `map_view.json`,
  bound to the PCD hash and same active session PID/process-start identity.
  Apply only a rigid display rotation through private TF; preserve PCDs, tiles,
  poses and extrinsics. Older maps need a measured map-frame upward vector or
  retain their frame. Do not apply current gravity to unrelated maps, fit floors,
  constrain estimation, or change calibration for a level-looking display.
  Check URDF axis/direction against measured gravity before claiming body
  alignment makes a map horizontal.

## 7. Verification and known limits

Choose checks according to the change:

- Documentation/naming: links, closed details, CLI help, installed configs and
  parsed YAML equivalence. Replay a representative recording when runtime paths
  or commands change; do not rerun every long bag just for prose edits.
- Estimator, filtering, timing, transport, concurrency, map I/O or localization:
  affected checks across all seven recordings and both families, following
  [the validation contract](doc/validation.md). Full qualification is 28 workflows
  (offline/online mapping/localization per recording), visualization, map saving,
  accepted trajectories and 1× online playback. Check reliable and best-effort
  publishers. If the user limits scope or inputs are unavailable, report that
  explicitly; do not count historical runs as fresh results.
- Hardware: stationary mapping/save/localization/display plus moving recorded
  data for trajectory visibility. Stationary success does not qualify moving
  routes, loop closure, large maps or runtime resource limits. Preserve firmware
  services and unrelated user sessions; stop only processes started by the test.

Use unique outputs. Wait for successful `/lightning/save_map` response before
stopping online mapping. Check `index.txt` and every tile, readable finite PCD
data and unchanged reference-map hashes after localization. Inspect actual pixels
and cloud geometry, not just window existence or PCD headers. A black nested X
desktop is not a rendered view. Keep gallery comparison and live-viewer checks
separate. Verify README `pcl_viewer` commands when changing export/visualization
dependencies and retain `pcl-tools` in the installer.

Capture exact commands, input counts, config/source/binary hashes, timings,
resource metric definitions, map/trajectory checks and failed trials. Never
rewrite old evidence for newer settings. Check complete input separately from
accepted/processed counts. A clean exit or plausible screenshot alone cannot pass
qualification; use the numerical criteria in `doc/validation.md`.

For timing checks, distinguish acquisition cadence, per-point spans, IMU coverage,
callback arrivals, processing intervals and actual rendered frame intervals.
Full Python CustomMsg decoding can backlog on ARM; use raw-CDR or C++ observers.
Do not compensate probe backlog with estimator timestamp offsets. Record the
Livox driver version and timestamp path: host-receipt fallback for unsynchronized
packets is not proof of hardware/PTP synchronization. A topic rate or configured
FPS cap does not establish smooth SSH rendering.

On RK3588, test estimation CPU 7 and RViz CPU 6 together with live sensors; record
queues, process memory, rendered pixels, frames/topics/QoS and display timing.
For RViz, check late subscribers, retained paths, valid scan/pose placement, map
hash preservation, malformed/empty maps, large messages and process cleanup.
Close all test viewers/servers/replays afterward without touching user sessions.

Library F retains visible revisit misalignment; there is no surveyed ground truth.
“Pass” means operational checks passed, not perfect geometry. A historical 1×
best-effort Building 3 trial missed one IMU sample without overflow; the unchanged
successful repeat does not erase that observation. Keep platform support separate
from measured hardware performance. New mountings, moving onboard routes,
resource limits and cross-host DDS require their own validation; historical
proposals are not completed tests.
