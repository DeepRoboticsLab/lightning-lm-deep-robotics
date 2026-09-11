# Validation and reproduction record

## 1. Scope and evidence

The reference qualification covers seven recordings, with offline mapping,
offline localization, online mapping, and online localization for each (28
workflows). It used Ubuntu 22.04 / ROS 2 Humble on an x86 workstation with 20 CPUs
and about 62 GiB RAM. Viewers rendered through Xvfb/Mesa; online replay ran at 1×.
Application subscriptions and the main qualification's bag publishers were
explicitly BEST_EFFORT. Every ROS participant used the supplied 64 MiB Fast DDS
SHM profile. Localization loaded each recording's freshly exported offline map;
online mapping independently exported its map through the save service.

The user separately verified Ubuntu 20.04 / Foxy and Ubuntu 22.04 / Humble support.
That platform verification is distinct from the detailed workstation measurements
below. RK3588 resource performance, fresh-OS dependency installation, physical
driver operation for a new mounting, and cross-host DDS were not measured here.

Compact evidence is kept inside the repository:

- [Reference results](validation/reference-results.json): exact historical
  application commands, config hashes, input counts, matching statistics,
  resource observations, and per-run assertions (28 core + 4 additional runs).
- [Source manifest](validation/reference-source-manifest.json): the 209-file
  source export used for the completed reference qualification, before this
  documentation/preset-name update.
- [Point-cloud integrity](validation/reference-cloud-integrity.tsv): 172 decoded
  PCD files from the core and additional checks, with point counts and bounds.
- [Preset provenance](validation/preset-provenance.json): current parsed presets,
  old/new hashes, and the equivalent Mid360 rename.

The immutable reference JSON still names the old `lite3_edu.yaml` file and uses
the original workstation paths. The production file is now `config/mid360.yaml`.
The rename and comments change its byte hash, but every parsed YAML setting is
unchanged; M20's preset is unchanged. Historical commands/hashes were preserved
rather than rewritten to imply a new run. Current runnable commands use the new
name. The reference manifest describes that snapshot, not every later source edit.

Raw maps, per-run logs, resources, actual viewer captures, and trajectories remain
in `/home/ubuntu/deep-robotics/reconstruction-evaluation/publish-validation/`.
These bulky artifacts are not required to build the repository. Do not delete
them during publication cleanup. The compact files above preserve the detailed
qualification results even when that local archive is unavailable.

## 2. Recordings and configurations

Local dataset root: `/home/ubuntu/deep-robotics/dataset`. The directory names
reflect the existing recordings and do not define the public supported hardware.

| Recording | Relative SQLite path | Current preset | Duration (s) | LiDAR / IMU messages |
|---|---|---|---|---|
| Library F | `m20_lidar_data/libraryf/libraryf_0.db3` | `m20_pro.yaml` | 1109.77 | 11098 / 213167 |
| Office | `m20_lidar_data/office/lidar_data_bag_0.db3` | `m20_pro.yaml` | 45.30 | 452 / 8301 |
| Building 1 | `lite3_lidar_data/building1/building1_0.db3` | `mid360.yaml` | 896.107 | 8961 / 179212 |
| Building 2 | `lite3_lidar_data/building2/building2_0.db3` | `mid360.yaml` | 169.03 | 1690 / 33804 |
| Building 3 | `lite3_lidar_data/building3/building3_0.db3` | `mid360.yaml` | 96.506 | 965 / 19302 |
| Grass 2 | `lite3_lidar_data/grass2/grass2_0.db3` | `mid360.yaml` | 193.356 | 1934 / 38672 |
| Road 1 | `lite3_lidar_data/road1/road1_0.db3` | `mid360.yaml` | 1420.638 | 14207 / 284123 |

One preset per family is used across all modes, without route-specific tuning.
Total input per mode is 39,307 LiDAR and 776,581 IMU messages. M20's headers include
53 stale LiDAR scans in Office and 191 in Library F; received counts remain full
while odometry consistently rejects these backwards headers.

## 3. Functional and reconstruction results

All 28 final workflows passed. PASS requires complete recorded sensor input,
clean exit, no queue overflow, actual rendered viewer pixels, and nonzero LIO
matches. Mapping also requires successful export, a complete tile index, and
readable finite PCDs. Localization requires initialization, finite monotonic
trajectories, normalized quaternions, confidence ≥1 for accepted matches,
unchanged map files, and online pose/TF publication. Tracking coverage requires
at least 99% accepted map matches and no accepted-pose gap over one second.

These checks establish functional replay behavior. They do not establish surveyed
trajectory accuracy, globally correct loop correspondences, or perfect geometry.
Library F retains visible revisit misalignment and remains the hardest case.

| Recording | Offline map points | Online map points | Offline accepted / attempted | Online accepted / attempted | Largest pose gap offline / online (s) |
|---|---|---|---|---|---|
| Library F | 1762406 | 1762406 | 5222 / 5222 | 5223 / 5223 | 0.408 / 0.408 |
| Office | 71494 | 71496 | 210 / 210 | 210 / 210 | 0.303 / 0.303 |
| Building 1 | 3307684 | 3307684 | 4384 / 4384 | 4382 / 4382 | 0.301 / 0.301 |
| Building 2 | 724227 | 724073 | 814 / 814 | 815 / 815 | 0.301 / 0.301 |
| Building 3 | 304280 | 306446 | 480 / 480 | 444 / 444 | 0.300 / 0.302 |
| Grass 2 | 314933 | 315768 | 966 / 966 | 871 / 871 | 0.300 / 0.304 |
| Road 1 | 3287938 | 3278746 | 7008 / 7008 | 6893 / 6893 | 0.301 / 0.304 |

Minor mode differences reflect scan timing, online buffered completion, and map
matching cadence; full input was checked separately. Accepted NDT confidence is
a diagnostic, not an accuracy percentage. The header image consists of seven
rendered reconstructions with no text; it is a qualitative gallery, separate
from the live-viewer captures in the qualification archive.

## 4. Resource observations

Memory is the peak sampled sum of process resident memory for the application
command and its children, including the `ros2 run` wrapper, sampled every 0.2 s.
The last column also includes the bag player when present. Shared pages can be
counted more than once; OS, display-server, other robot processes, sensor drivers,
and hardware graphics allocations are excluded. These are workstation
observations, not RK3588 real-time or RAM guarantees.

| Recording | Offline mapping (MiB) | Offline localization (MiB) | Online mapping (MiB) | Online localization (MiB) | Peak app + player (MiB) |
|---|---|---|---|---|---|
| Library F | 1058.7 | 750.6 | 1181.5 | 918.3 | 1229.5 |
| Office | 653.9 | 394.4 | 774.9 | 474.1 | 1058.3 |
| Building 1 | 1077.3 | 916.6 | 1240.5 | 1041.4 | 1240.5 |
| Building 2 | 519.4 | 424.3 | 633.4 | 512.2 | 700.6 |
| Building 3 | 401.2 | 389.2 | 528.3 | 472.4 | 646.1 |
| Grass 2 | 408.8 | 394.3 | 520.6 | 475.7 | 635.1 |
| Road 1 | 1068.3 | 1124.3 | 1183.0 | 1286.4 | 1440.4 |

Largest M20 application mapping/localization observations are 1.15/0.90 GiB;
Mid360 observations are 1.21/1.26 GiB. The largest application-plus-player sum is
1.41 GiB. NDT, TBB, passive OpenMP waiting, sensor-queue bounds, and 5 Hz matching
are documented in [implementation.md](implementation.md). Point filtering and
0.5 m mapping voxels were preserved while adding those runtime controls.

Headless exported-build Office mapping/localization also passed with DISPLAY
and WAYLAND_DISPLAY unset. `/usr/bin/time -v` maximum single-process RSS was
145416/142768 KiB respectively. This is a different metric from the sampled
process-tree sums above and must not be compared as if measured identically.

## 5. Reproducing and extending the checks

Build and source using the README, choose a row from section 2, then set paths
from the repository root, for example:

```bash
source scripts/setup.bash
export CONFIG="$PWD/config/mid360.yaml"
export BAG="/home/ubuntu/deep-robotics/dataset/lite3_lidar_data/building3/building3_0.db3"
export MAP="$PWD/data/building3_check"
```

Use the four README commands, one workflow at a time. Choose a new map output
and trajectory path for every trial. Online save output is controlled by the
service's `map_id`, relative to the node's working directory. The reference
qualification ran each workflow from a unique output directory and passed
absolute config, bag, and offline-map paths. Use separate ROS domain IDs if
independent replays must run concurrently, and record machine load.

For explicit best-effort publishers, put this temporary override in the run
directory (use `/LIDAR/POINTS` and `/IMU` for M20), then add
`--qos-profile-overrides-path /absolute/path/to/qos.yaml` to bag playback:

```yaml
/livox/lidar:
  reliability: best_effort
  durability: volatile
  history: keep_last
  depth: 16
/livox/imu:
  reliability: best_effort
  durability: volatile
  history: keep_last
  depth: 1000
```

The original Humble qualification used `--clock --delay 5 --rate 1.0` in addition
to these overrides. The public command now uses the shared Foxy/Humble
`--rate 1.0` interface; the estimator uses sensor acquisition times and does not
require `/clock`. The separate rename/command checks record the updated command.
Do not force ROS time flags into the Foxy instructions: its stock player lacks
the Humble `--clock`/`--delay` arguments.

For every run record the exact source/configuration and effective binary,
input identity/counts, command, UI and thread settings, exit status, output paths,
queue warnings, accepted/rejected matches, map/trajectory checks, and resource
measurement method. Inspect actual viewer pixels and final map geometry. Check
`index.txt` references and decode the PCD data; a header alone cannot establish
finite points. Hash reference files before/after localization. Check online
`/lightning/pose` and `/tf` with the application running.

The existing external helpers `validate.py`, `final_report.py`, `check_headless.py`,
`check_errors.py`, and queue/cloud check programs remain in the local archive.
They are evaluation aids, not published product dependencies. Some contain old
configuration names or paths; use a new run wrapper and the current preset name,
never overwrite historical results. No automated geometric-ground-truth metric
or broad board-level benchmark is implied by these helpers.

## 6. Build and additional validation

`final-root-clean-build.log` records rebuilding after archiving/removing generated
package outputs. `final-export-build.log` records a separate source-only export
with its own Pangolin/.deps installation. The source-only bootstrap used the
bundled ZIP, exposed all four executables and SaveMap/Livox interfaces, and did
not depend on another checkout's `.deps`. The dependency installer passed with
required packages already installed; fresh OS apt installation was not exercised.

`final-readme-check/` contains four exported-build normal-rate online checks:
mapping and localization on Office and Building 3 with default reliable bag
publishers. They passed complete delivery, rendered viewers, exports,
pose/TF, and unchanged maps. `final-readme-qos.log` and
`final-readme-livox-qos.log` observed RELIABLE publishers and BEST_EFFORT
subscribers for both message formats. Its offline-map directory is a symlink to
the core map outputs, not evidence of seven additional exported-build map runs.

`queue-check.log` covers count/weight eviction, object release, ordering, drain,
and shutdown. `error-checks/results.json` covers missing input/map failures,
nonempty-output preservation, empty save response 3, and invalid-ID response 2.
`headless-check/` contains the headless Office checks. These artifact names are
relative to the external archive described in section 1.

## 7. Publication update checks

The current source also passed a fresh Humble build using the README installer
and build script, starting from a separate source-only export and the bundled
Pangolin ZIP. All four executables load, expose the updated CLI help, resolve
their libraries independently of the working checkout, and install the expected
interfaces/configurations and ROS-distribution marker. See
[build checks](validation/build-checks.json) and the
[matching runtime-source manifest](validation/publication-runtime-manifest.json).
The package build took 2 min 19 s after Pangolin was built. The workstation's
existing OpenCV dependency links legacy TBB alongside oneTBB and emitted a linker
warning; the build and runtime checks passed. This environment detail belongs in
the validation record rather than a claim about every Ubuntu installation.

The Mid360 naming update preserved every parsed sensor/estimator setting. The
package description and all four applications' CLI help now name M20 Pro and
Mid360. Only the two current YAML presets and Fast DDS XML remain in the source
and installed configuration directories.

The README's playback command, `ros2 bag play "$BAG" --rate 1.0`, was exercised
without `--clock`, `--delay`, or QoS overrides. Fresh Office/Mid360 Building 3
runs passed all four applications (eight checks), including full sensor input,
actual live viewers, no queue overflow, save-service export, 18 readable finite
PCDs, valid trajectories, online pose/TF output, and unchanged reference maps.
All attempted localization matches in these runs were accepted. The full
28-workflow reference suite was not rerun solely for the naming/documentation
change; its original settings and evidence remain preserved above.

[Publication check results](validation/publication-checks.json) contain the
commands and assertions. [Decoded cloud checks](validation/rename-cloud-integrity.tsv)
record all 18 PCDs. [Recording/config checks](validation/recording-config-checks.json)
confirm that the current presets match all seven bags' topics/types and the
reference parameter snapshots. [Script checks](validation/script-checks.json)
exercise distro-specific dependency selection and setup/mismatch handling in
temporary fixtures; they are shell behavior checks, not native Foxy build runs.

Saved-map viewing was checked separately with `pcl_viewer <global.pcd> -ps 2` on
all seven reference offline maps and all seven reference online maps. All
fourteen produced rendered map pixels and preserved the input files. These
interactive viewers were terminated by the test harness after capture, rather
than counted as applications that naturally exit at bag completion. See
[PCL viewer results](validation/pcl-viewer-checks.json). The dependency installer
now includes `pcl-tools`, already installed on this workstation. Actual viewer
captures and logs are retained under the external `saved-map-viewer-check/`
archive. Test X servers were closed without affecting the user's desktop.

## 8. Retained failures and experiment context

A preceding 1× Building 3 online localization run received all 965 LiDAR messages
but 19,301 of 19,302 IMU messages. There was no application queue overflow and
all 444 map matches were accepted. Repeating without code, QoS, or buffer changes
received full input. The original is retained under `normal-rate-imu-loss/`.
The final table reports the passing repeat without claiming guaranteed delivery.

Earlier accelerated concurrent 4× and 2× runs overflowed bounded queues before
the final TBB/passive-waiting controls; see `bounded-4x-stress/` and
`bounded-2x-stress/`. Software rendering also created substantial CPU contention.
Those trials do not establish normal-rate failure or current accelerated-rate
capability. Several trials were interrupted, not passed.

Other historical folders include `pre-match-validation/` (unconditional-success
localization), `synchronous-map-trial/`, `dense-trial/`, `registration-trial/`,
`voxel-trial/`, and `resource-mapping/`. Do not use `verified/loc_offline` as final
evidence. The accepted core outputs are `final-verified/<mode>/<recording>/`.
The gallery's original clouds/images remain in the ignored
`outputs/seven-datasets-unified-20260910/` directory.

The [archived Library F proposal](validation/libraryf-proposal.md) preserves the
original experiment plan inside the repository. Its reference was a
conversation/image rather than a recovered original cloud.
It proposed historical reproduction, active-edge rejection, solver determinism,
optimization timing, loop validation, keyframe thresholds, frontend comparisons,
and timing diagnostics. It also proposed repeatability and wall/revisit geometry
measurements. A proposal is not evidence that every experiment or repetition was
completed; the retained results above define what was actually qualified.

For renewed geometry work, preserve the original output, compare raw odometry
against loop-corrected maps to locate the first distortion, and change one
hypothesis at a time. Use fixed camera/crop/voxel settings and one global rigid
alignment for each whole result. Measure wall thickness/curvature, corner
orientation, repeated-surface separation, revisit corrections, point support,
coverage, runtime, and variability. Do not snap walls, independently align each
surface, flatten z, smooth the evaluated trajectory, or suppress difficult data
to make a picture look better. No surveyed ground-truth errors or perfect
reconstruction should be claimed from these operational checks.
