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
and `scripts/setup.bash`. Keep experimental runners, screenshot/render scripts,
profiling helpers, temporary test programs, and unrelated configs out of the
published tree. Store lasting maintenance knowledge and compact verification
records in `doc/`; keep bags, generated maps, raw logs, and binaries in ignored
or external output directories. Do not delete existing evidence during cleanup.

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
RK3588 target start with `CMAKE_BUILD_PARALLEL_LEVEL=2`. This is a build-memory
choice; runtime threading is controlled separately. Do not enable x86 flags on
ARM or make `-march=native` a portable binary default.

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
physical drivers and cross-host UDP still require validation.

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
