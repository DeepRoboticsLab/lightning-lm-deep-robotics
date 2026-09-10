# Libraryf reproduction with live visualization

This local branch uses the March 18, 2026 upstream reconstruction implementation
from `f5eded0c3f5aecf4761476bf733e663bd714c0ee`, with diagnostic exports and the
tested Pangolin shutdown cleanup. Existing Ubuntu 22/Humble build compatibility
is retained. The previous implementation remains on `ysc-20260909` at `0d38bbc`.

Build with all available CPUs:

```bash
bash scripts/build_libraryf.sh
```

From the graphical desktop session, replay the libraryf bag with the live 3D viewer:

```bash
bash scripts/run_libraryf_offline.sh
```

The launcher defaults to
`../dataset/m20_lidar_data/libraryf/libraryf_0.db3`. An optional first argument
selects another `.db3` bag with the same sensor topics and conventions.
It sources ROS Humble, uses the local `bin/run_slam_offline` and `libraryf-build`
libraries, and keeps the tested eight-CPU affinity and four OpenMP threads.
Compilation uses every available CPU.

`config/libraryf_march18_3d.yaml` enables the viewer and loop closing, disables
the height prior, and records diagnostics. Each invocation creates a new
`outputs/libraryf-<timestamp>-<suffix>/` directory. Its saved point cloud is
`data/new_map/global.pcd`; configuration, logs, timing and pose/cloud diagnostics
are retained alongside it. The viewer closes when processing finishes.

This is the closest tested 3D reproduction, not an exact recovery of the
photographed reference. The three offline historical runs have a roughly
3 m north-return surface separation, and live-view backend results can vary.
Do not enable a fixed-height prior to conceal that error.

The complete investigation remains at
`/home/ubuntu/deep-robotics/reconstruction-evaluation/reports/final-report.md`.
