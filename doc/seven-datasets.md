# Seven local dataset replays

The reconstruction implementation at `313773c1f172afdd859a591703a1facd3ca0cf20`
uses one executable and two shared sensor presets. All five Lite3 recordings use
the Mid360 preset; Library F and Office use the M20 preset. There is no per-route
tuning, fixed-height prior, or post-processing that flattens the maps.

The comparison reference is
[`shengxian886` at `5002b30`](https://github.com/DeepRoboticsLab/lightning-lm-deep-robotics/commit/5002b3099e192b50818e0c030731b8b018f61ebf).
Its complete frontend works much better on the Mid360 recordings than the old
unconditional IMU-filter setup, but replacing the entire core worsens Library F
with height unconstrained. The selected implementation retains the historical
M20 path and adds a configurable Mid360 motion model, keyframe spacing and graph
optimization schedule. Both models live in the same code and executable.

## Configurations

| Datasets | Shared configuration | Input topics |
| --- | --- | --- |
| M20: `libraryf`, `office` | [libraryf_march18_3d.yaml](../config/libraryf_march18_3d.yaml) | `/LIDAR/POINTS`, `/IMU` |
| Lite3 / Mid360: `building1`, `building2`, `building3`, `grass2`, `road1` | [lite3_livox_3d.yaml](../config/lite3_livox_3d.yaml) | `/livox/lidar`, `/livox/imu` |

| Active setting | M20 | Lite3 / Mid360 |
| --- | --- | --- |
| LiDAR input | PointCloud2, historical RoboSense handler (`lidar_type: 3`) | Livox CustomMsg (`lidar_type: 1`) |
| Prediction model | Inertial | Constant velocity plus gyroscope rotation |
| Gyroscope smoothing / rate limiter | On | Off |
| Keyframe displacement / angle | 2 m / 15° | 1 m / 10° |
| Blind distance setting | 0.5 m | 0.1 m |
| Keep every Nth point | 6 | 4 |
| Scan / local-map voxel size | 0.5 / 0.5 m | 0.5 / 0.5 m |
| LiDAR-to-IMU translation | `[0, 0, 0]` m | `[0, 0, 0.28]` m |
| LiDAR-to-IMU rotation | Identity | Identity |
| Extrinsic estimation | Off | Off |
| Loop candidate radius | 60 m | 20 m |
| Query/history keyframe gap | 20 | 50 |
| NDT acceptance score | >1.0 | >1.3 |
| Graph optimization | Every keyframe, preserving historical behavior | When a new loop passes the score threshold |
| Height constraint | **Off** | **Off** |
| Live 3D visualization / loop closing | On / on | On / on |
| 2D grid generation | Off | Off |

Both presets use four ESKF iterations, no Anderson acceleration and no requested
scan skipping. The runner changes only the diagnostic output directory between
runs in each family. Exact executed YAML files and the source presets are saved
with the outputs. Extrinsics come from the supplied robot presets and were not
independently calibrated here.

The `lidar_type: 3` value is specific to this historical reconstruction core:
its handler reads the M20 RoboSense fields. The linked newer branch instead
calls that sensor type `4`; copying that value into this core is incorrect.
The localization and ROI sections retained in the YAML are not a rendering crop.
This core's offline preprocessing does not apply those height ROI fields.

## Final full replays — 2026-09-10

| Dataset | Replay time | Saved points | Keyframes | Zero-match scans | Max implied speed |
| --- | ---: | ---: | ---: | ---: | ---: |
| Library F | 101.25 s | 1,762,406 | 718 | 0 | 2.16 m/s |
| Office | 9.50 s | 71,496 | 39 | 0 | 1.22 m/s |
| Building 1 | 65.11 s | 3,307,684 | 1,238 | 0 | 1.87 m/s |
| Building 2 | 10.99 s | 723,561 | 258 | 0 | 1.81 m/s |
| Building 3 | 6.35 s | 304,396 | 138 | 0 | 1.78 m/s |
| Grass 2 | 8.11 s | 314,797 | 246 | 0 | 1.65 m/s |
| Road 1 | 71.19 s | 3,292,086 | 1,968 | 0 | 1.94 m/s |

All seven processes reached the end of their bags, saved maps and exited
normally with visualization enabled. Both M20 raw scan-pose CSVs are
byte-identical to the earlier replay. The optimized maps can still differ
because of the existing parallel backend; raw-pose equality does not imply
byte-identical global clouds. The large Mid360 divergence and end-of-recording
pose jumps are absent in these replays.

## Why the Mid360 change helps

The old core ignored `imu_filter: false` and always smoothed and rate-limited the
gyroscope. Honoring the setting fixes much of the error, but acceleration-driven
prediction still produces bad tails in Building 1 and Road 1. The reference
branch's 12-state frontend omits velocity integration in its prediction mean.
The selected implementation makes constant-velocity prediction an explicit
option and keeps its mean, covariance Jacobian and deskew model consistent:

```
p_dot = v
R_dot = R * skew(gyro - gyro_bias)
v_dot = 0 + process_noise
```

LiDAR observations continue to estimate all three components of position,
rotation and velocity. Accelerometer readings initialize the gravity direction,
but do not drive translation in this mode; accelerometer bias and gravity have
no acceleration coupling in the covariance. Unmodelled acceleration still
increases velocity uncertainty through process noise. This is a motion prior
between measurements, not a constant-speed constraint on the complete route.
The M20 preset keeps normal inertial prediction.

A compiled analytic check verifies inertial acceleration integration, the
constant-velocity mean and covariance, gyroscope rotation, process noise and a
rising 3D trajectory estimated from synthetic LiDAR observations:

```bash
source /opt/ros/humble/setup.bash
cmake --build libraryf-build --target test_lio_motion_model --parallel "$(nproc)"
./bin/test_lio_motion_model
```

## Comparison evidence

The initial replay archive is `outputs/seven-datasets-20260910-v2/`. It is
preserved, including the failed geometries. The full branch reference is
`outputs/seven-datasets-shengxian886-20260910/`; its supplied sensor settings were
retained, with the viewer enabled, optional 2D grid generation disabled, and the
M20 height prior disabled for the 3D comparison. That branch applies input
height ROI filtering, unlike the selected historical core.

The intermediate controls are:

| Mid360 variant | Building 1 max implied speed | Building 2 | Road 1 |
| --- | ---: | ---: | ---: |
| Original historical core | 7053.83 m/s | 23.04 m/s | 619.59 m/s |
| Honor `imu_filter: false` only | 36.05 m/s | 2.95 m/s | 593.08 m/s |
| Consistent constant-velocity prediction, historical keyframe spacing | 1.87 m/s | 1.79 m/s | 1.96 m/s |
| Full `shengxian886` reference | 1.89 m/s | 1.79 m/s | 2.92 m/s |

These are raw keyframe displacements divided by timestamp intervals, **not
measured robot velocities or ground-truth errors**. Zero-match scans and
implausible speeds expose gross tracking failures; their absence alone does not
establish accuracy. Full saved geometry and reference trajectories were also
compared. Library F remains the difficult sequence and retains the earlier
revisit-alignment limitation; this work does not certify perfect geometry.

The final replay restores the reference preset's finer Mid360 keyframe spacing
and runs graph optimization when loop candidates pass the score threshold. Its
archive is `outputs/seven-datasets-unified-20260910/`. See its `results.md`,
`results.csv`, `results.json` and per-run diagnostics for the actual measurements,
point clouds, images and executed configurations. The comparison work and
synthetic test log are under
`../reconstruction-evaluation/seven-datasets-comparison-20260910/`.

## Run all seven

From the repository root on Ubuntu 22.04 with ROS Humble:

```bash
bash scripts/build_libraryf.sh
python3 scripts/run_all_local_datasets.py
```

The build uses all available CPUs and includes the Livox Fast DDS type-support
library required to deserialize CustomMsg bags. Each replay uses eight CPUs,
four OpenMP threads and the live viewer. The default dataset root is
`../dataset`; override it with `--data-root /path/to/dataset`. `--dataset NAME`
can select one recording; repeat it to select several. `--no-ui` disables the
viewer. `--source`, `--build`, `--m20-config` and `--lite3-config` support isolated
version comparisons.

Each new batch directory contains the archived executable and project libraries,
source revision and patch, bag hashes, exact YAML, logs, resource usage,
trajectories, local keyframe clouds and `data/new_map/global.pcd`. Existing
replays are never overwritten.

## Render the maps

The renderer needs NumPy, SciPy, Matplotlib and python-lzf. Gravity export uses
the installed ROS Humble Python message support:

```bash
source /opt/ros/humble/setup.bash
python3 scripts/export_gallery_gravity.py outputs/seven-datasets-<timestamp>
python3 scripts/render_dataset_gallery.py outputs/seven-datasets-<timestamp>
python3 scripts/render_dataset_gallery.py outputs/seven-datasets-<timestamp> \
  --overview doc/images/seven-datasets-overview.png
```

Use a Python environment containing the plotting dependencies for the latter
two commands. Original global PCDs are copied unchanged to `point_clouds/`.
Each individual image is a text-free 2400 × 1600 oblique view. The text-free
6000 × 2400 overview has Library F at left, with Office / Building 1 / Building 2
across the upper right and Building 3 / Grass 2 / Road 1 across the lower right.
There are no titles, labels, scale bars, legends or decorative frames in the
images. Dataset identification and qualifications stay in this document.

Rendering uses all finite saved map points, deterministic 0.2 m display voxels
and no height crop. One rigid transform removes the first-keyframe graph gauge
and makes the first scan's supported floor normal vertical. The floor fit
uses points within 20 m, a 6 cm inlier threshold, a 40° initial-IMU direction
gate and a plane 0.05–3 m below the sensor. It requires at least 100 points and
6% support; otherwise the initialized IMU direction is used. The initial IMU
estimate can be tilted relative to the floor (about 22° in Building 1), so it is
not a reliable camera direction in every recording. This is one global display
rotation, not a trajectory correction or height constraint. A camera yaw and
per-panel fit select the view. Colors encode relative height with independent 2nd/98th
percentile color limits; only the colors saturate, not the geometry. Opaque
occlusion selects the nearest point per projected pixel. Every transform, range,
checksum and camera setting is recorded in `images/*-render.json`. No wall
snapping, local alignment, geometric scaling or flattening is used.
