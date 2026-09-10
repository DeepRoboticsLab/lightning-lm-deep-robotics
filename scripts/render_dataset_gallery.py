#!/usr/bin/env python3
"""Render real saved PCD maps and a seven-result README overview; no geometry warping."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import re
import shutil
import struct

import lzf
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation

ORDER = ['libraryf', 'office', 'building1', 'building2', 'building3', 'grass2', 'road1']
TITLES = {'libraryf': 'Library F', 'office': 'Office', 'building1': 'Building 1',
          'building2': 'Building 2', 'building3': 'Building 3', 'grass2': 'Grass 2', 'road1': 'Road 1'}
BG, INK, MUTED = '#f3f6fa', '#13243a', '#627187'
plt.rcParams.update({'font.family': 'DejaVu Sans', 'text.color': INK, 'axes.labelcolor': INK})


def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda: f.read(8 * 1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def read_pcd(path):
    with path.open('rb') as f:
        header = {}
        while True:
            line = f.readline()
            if not line:
                raise ValueError(f'Incomplete PCD header: {path}')
            line = line.decode('ascii').strip()
            if not line or line.startswith('#'):
                continue
            key, *value = line.split()
            header[key] = value
            if key == 'DATA':
                break
        names, sizes = header['FIELDS'], list(map(int, header['SIZE']))
        kinds, counts = header['TYPE'], list(map(int, header.get('COUNT', ['1'] * len(names))))
        n = int(header['POINTS'][0])
        types = {'F': 'f', 'I': 'i', 'U': 'u'}
        dtype = np.dtype([(k, '<' + types[t] + str(s), (c,)) if c > 1 else (k, '<' + types[t] + str(s))
                          for k, s, t, c in zip(names, sizes, kinds, counts)])
        mode = header['DATA'][0]
        if mode == 'binary_compressed':
            compressed, original = struct.unpack('<II', f.read(8))
            raw = lzf.decompress(f.read(compressed), original)
            if len(raw) != original:
                raise ValueError('PCD decompression size mismatch')
            a, offset = np.empty(n, dtype), 0
            for name, size, kind, count in zip(names, sizes, kinds, counts):
                field = np.frombuffer(raw, dtype='<' + types[kind] + str(size), count=n * count, offset=offset)
                a[name] = field.reshape((n, count) if count > 1 else (n,))
                offset += n * size * count
        elif mode == 'binary':
            a = np.frombuffer(f.read(), dtype=dtype, count=n)
        elif mode == 'ascii':
            rows, a, offset = np.loadtxt(f, ndmin=2), np.empty(n, dtype), 0
            for name, count in zip(names, counts):
                a[name] = rows[:, offset:offset + count].reshape((n, count) if count > 1 else (n,))
                offset += count
        else:
            raise ValueError(f'Unsupported PCD encoding: {mode}')
    return np.column_stack([a['x'], a['y'], a['z']]).astype(np.float64)


def voxel(x, size):
    cells = np.floor(x / size).astype(np.int64)
    order = np.lexsort((x[:, 2], x[:, 1], x[:, 0], cells[:, 2], cells[:, 1], cells[:, 0]))
    cells = cells[order]
    keep = np.r_[True, np.any(cells[1:] != cells[:-1], axis=1)]
    return x[order[keep]]


def pose(row, prefix):
    t = np.eye(4)
    t[:3, :3] = Rotation.from_quat([float(row[prefix + k]) for k in ['qx', 'qy', 'qz', 'qw']]).as_matrix()
    t[:3, 3] = [float(row[prefix + k]) for k in ['x', 'y', 'z']]
    return t


def tracking_diagnostics(run):
    """Flag observed tracking anomalies, without claiming ground-truth accuracy."""
    with (run / 'diagnostics/keyframes.csv').open() as f:
        rows = list(csv.DictReader(f))
    timestamps = np.array([float(r['timestamp']) for r in rows])
    positions = np.array([[float(r['raw_' + k]) for k in 'xyz'] for r in rows])
    steps = np.linalg.norm(np.diff(positions, axis=0), axis=1)
    intervals = np.diff(timestamps)
    speeds = np.divide(steps, intervals, out=np.zeros_like(steps), where=intervals > 0)
    max_speed = float(speeds.max(initial=0))
    runaway = np.flatnonzero(speeds > 100)
    first_runaway = float(timestamps[runaway[0] + 1] - timestamps[0]) if len(runaway) else None
    late = first_runaway is not None and first_runaway > .8 * (timestamps[-1] - timestamps[0])
    matches = [int(n) for n in re.findall(r'\[ mapping \].*effect num\s*:\s*(\d+)', (run / 'run.log').read_text())]
    streak = longest = 0
    for n in matches:
        streak = streak + 1 if n == 0 else 0
        longest = max(longest, streak)
    lost = longest >= 50 or matches.count(0) > max(100, .1 * len(matches))
    if lost:
        label = 'TRACKING LOST · divergent output'
    elif len(runaway):
        label = 'LATE DIVERGENCE' if late else 'DIVERGENT OUTPUT'
    elif max_speed > 20:
        label = 'LARGE POSE JUMPS'
    else:
        label = ''
    return {'zero_match_scans': matches.count(0), 'longest_zero_match_streak': longest,
            'tracking_lost': lost, 'quality_label': label,
            'raw_keyframe_max_implied_speed_mps': max_speed,
            'raw_keyframe_max_step_m': float(steps.max(initial=0)),
            'raw_keyframe_path_length_m': float(steps.sum()),
            'nonpositive_keyframe_intervals': int((intervals <= 0).sum()),
            'first_speed_over_100mps_elapsed_seconds': first_runaway,
            'tracking_flag_rule': 'Tracking lost: >=50 consecutive zero-match scans, or >10% zero-match scans and >100 such scans. Divergence: raw keyframe displacement/time >100 m/s; late if first occurrence is after 80% of keyframe duration. Large pose jumps: >20 m/s. These are diagnostic flags, not measured ground-truth error; unflagged maps may still drift.'}


def display_up_direction(run, first, gravity):
    """Choose one camera up direction from the first scan's floor, without warping geometry."""
    imu_up = np.array(gravity['initial_up_direction'])
    raw_rotation = pose(first, 'raw_')[:3, :3]
    prior = raw_rotation.T @ imu_up
    cloud = read_pcd(run / 'diagnostics/clouds' / (first['id'] + '.pcd'))
    points = cloud[np.isfinite(cloud).all(axis=1) & (np.linalg.norm(cloud, axis=1) < 20)]
    method = {'method': 'First-scan floor plane defines one global display rotation; initial IMU direction is the fallback.',
              'range_m': 20, 'distance_threshold_m': .06, 'max_imu_angle_deg': 40,
              'floor_distance_limits_m': [.05, 3], 'seed': 7, 'trials': 1500}
    if len(points) < 100:
        return imu_up, dict(method, selected='imu', reason='Insufficient first-scan support')
    rng = np.random.default_rng(7)
    points = points[rng.choice(len(points), min(3000, len(points)), replace=False)]
    best_count, best = 0, None
    for _ in range(1500):
        a, b, c = points[rng.choice(len(points), 3, replace=False)]
        normal = np.cross(b - a, c - a)
        length = np.linalg.norm(normal)
        if length < 1e-6:
            continue
        normal /= length
        if normal @ prior < 0:
            normal = -normal
        distance = -normal @ a
        if normal @ prior < np.cos(np.radians(40)) or not .05 < distance < 3:
            continue
        inliers = abs(points @ normal + distance) < .06
        count = int(inliers.sum())
        if count > best_count:
            best_count, best = count, inliers
    if best_count < max(100, .06 * len(points)):
        return imu_up, dict(method, selected='imu', reason='No supported floor plane')
    support = points[best]
    center = support.mean(axis=0)
    _, _, vt = np.linalg.svd(support - center, full_matrices=False)
    normal = vt[-1]
    if normal @ prior < 0:
        normal = -normal
    up = raw_rotation @ normal
    return up, dict(method, selected='floor', samples=len(points), support=best_count,
                    normal_first_cloud=normal.tolist(), up_raw_world=up.tolist(),
                    floor_distance_m=float(-normal @ center),
                    fit_rms_m=float(np.sqrt(np.mean(((support - center) @ normal) ** 2))),
                    imu_angle_deg=float(np.degrees(np.arccos(np.clip(up @ imu_up, -1, 1)))))


def prepare(batch, name, size):
    run = batch / name
    manifest = json.loads((run / 'manifest.json').read_text())
    if manifest['status'] != 'complete':
        raise ValueError(f'{name} did not complete; refusing to display it as a successful reconstruction')
    cloud = run / 'data/new_map/global.pcd'
    points = read_pcd(cloud)
    finite = np.isfinite(points).all(axis=1)
    count = len(points)
    points = points[finite]
    with (run / 'diagnostics/keyframes.csv').open() as f:
        first = next(csv.DictReader(f))
    # Remove one global pose-graph gauge, using the corresponding first keyframe.
    transform = pose(first, 'raw_') @ np.linalg.inv(pose(first, 'opt_'))
    gravity = json.loads((run / 'initial-gravity.json').read_text())
    # Choose one floor-level camera frame, without flattening any map points.
    up, orientation = display_up_direction(run, first, gravity)
    if up[2] < -1 + 1e-12:
        level = Rotation.from_rotvec([np.pi, 0., 0.])
    else:
        level = Rotation.from_quat([up[1], -up[0], 0., 1 + up[2]])
    level_transform = np.eye(4)
    level_transform[:3, :3] = level.as_matrix()
    transform = level_transform @ transform
    points = points @ transform[:3, :3].T + transform[:3, 3]
    points = voxel(points, size)
    # A camera yaw only: put the longer horizontal extent across the image.
    covariance = np.cov(points[:, :2], rowvar=False)
    _, vectors = np.linalg.eigh(covariance)
    axis = vectors[:, -1]
    if axis[0] < 0:
        axis *= -1
    yaw = -np.arctan2(axis[1], axis[0])
    camera_rotation = Rotation.from_euler('z', yaw).as_matrix()
    points = points @ camera_rotation.T
    color_limits = np.percentile(points[:, 2], [2, 98])
    if color_limits[1] - color_limits[0] < .01:
        color_limits[1] = color_limits[0] + .01
    stats = {'dataset': name, 'title': TITLES[name], 'family': manifest['family'],
             'points_full': count, 'nonfinite_points': int((~finite).sum()), 'points_display': len(points),
             'voxel_m': size, 'height_crop': None, 'color_percentiles': [2, 98],
             'raster_occlusion': 'Nearest point per projected pixel at 200 dpi',
             'color_limits_m': color_limits.tolist(), 'whole_map_rigid_transform': transform.tolist(),
             'camera_yaw_deg': float(np.degrees(yaw)), 'oblique_elevation_deg': 65,
             'initial_gravity': gravity,
             'display_orientation': orientation,
             'full_display_bounds_m': [points.min(axis=0).tolist(), points.max(axis=0).tolist()],
             'elapsed_seconds': manifest['elapsed_seconds'], 'recording_seconds': manifest['bag']['duration_seconds'],
             'keyframes': manifest['counts']['keyframes'], 'mapping_scans': manifest['counts']['mapping_scans'],
             **tracking_diagnostics(run),
             'source_pcd_sha256': sha(cloud), 'config_source': manifest['config_source'],
             'method': 'Full saved cloud, finite points, deterministic display voxels, one rigid first-keyframe/floor transform and camera yaw. No height crop, local alignment, wall snapping or smoothing. Colors are relative height, normalized per map. Original saved PCD is unchanged.'}
    images = batch / 'images'
    images.mkdir(exist_ok=True)
    np.savez_compressed(images / (name + '-display.npz'), xyz=points)
    (images / (name + '-render.json')).write_text(json.dumps(stats, indent=2) + '\n')
    export = batch / 'point_clouds'
    export.mkdir(exist_ok=True)
    shutil.copy2(cloud, export / (name + '.pcd'))
    return points, stats


def draw_cloud(ax, points, stats, elevation=65, point_size=.28, bar=True):
    rad = np.radians(elevation)
    u = points[:, 0]
    v = points[:, 1] * np.sin(rad) + points[:, 2] * np.cos(rad)
    depth = -points[:, 1] * np.cos(rad) + points[:, 2] * np.sin(rad)
    lo, hi = np.array([u.min(), v.min()]), np.array([u.max(), v.max()])
    span = np.maximum(hi - lo, .1)
    box = ax.get_position()
    ratio = box.width * ax.figure.get_figwidth() / (box.height * ax.figure.get_figheight())
    half = span * .56
    if half[0] / half[1] < ratio:
        half[0] = half[1] * ratio
    else:
        half[1] = half[0] / ratio
    center = (hi + lo) / 2
    view_lo, view_hi = center - half, center + half
    ax.set(xlim=[view_lo[0], view_hi[0]], ylim=[view_lo[1], view_hi[1]])
    ax.set_aspect('equal')
    ax.set_axis_off()
    # Resolve opaque point occlusion at the output pixel scale. This avoids
    # drawing millions of points onto the same pixels in a divergent map.
    pixels = np.maximum(1, np.rint([box.width * ax.figure.get_figwidth() * 200,
                                    box.height * ax.figure.get_figheight() * 200])).astype(int)
    ix = ((u - view_lo[0]) / (view_hi[0] - view_lo[0]) * pixels[0]).astype(np.int64)
    iy = ((v - view_lo[1]) / (view_hi[1] - view_lo[1]) * pixels[1]).astype(np.int64)
    nearest_first = np.argsort(-depth, kind='stable')
    _, first = np.unique((iy * pixels[0] + ix)[nearest_first], return_index=True)
    visible = nearest_first[first]
    order = visible[np.argsort(depth[visible], kind='stable')]
    ax.scatter(u[order], v[order], c=points[order, 2], s=point_size, cmap='turbo',
               vmin=stats['color_limits_m'][0], vmax=stats['color_limits_m'][1], linewidths=0, rasterized=True)
    if bar:
        target = span[0] * .18
        candidates = 10 ** np.floor(np.log10(target)) * np.array([1., 2., 5., 10.])
        length = candidates[np.argmin(abs(np.log(candidates / target)))]
        x, y = view_lo + (view_hi - view_lo) * .045
        ax.plot([x, x + length], [y, y], color=INK, lw=1.6, solid_capstyle='butt')
        label = f'{length / 1000:g} km' if length >= 1000 else f'{length:g} m'
        ax.text(x + length / 2, y, label, ha='center', va='bottom', fontsize=10, color=MUTED)


def standalone(batch, name, points, stats):
    """A clean, text-free oblique map panel for publication layouts."""
    fig = plt.figure(figsize=(12, 8), facecolor='white')
    ax = fig.add_axes([0, 0, 1, 1])
    draw_cloud(ax, points, stats, elevation=65, point_size=.35, bar=False)
    fig.savefig(batch / 'images' / (name + '.png'), dpi=200, facecolor='white')
    plt.close(fig)


def write_results(batch):
    metadata = json.loads((batch / 'batch.json').read_text())
    results = []
    for name in ORDER:
        stats = json.loads((batch / 'images' / (name + '-render.json')).read_text())
        manifest = json.loads((batch / name / 'manifest.json').read_text())
        results.append({'dataset': name, 'family': stats['family'], 'execution': manifest['status'],
                        'elapsed_seconds': round(stats['elapsed_seconds'], 2),
                        'points': stats['points_full'], 'keyframes': stats['keyframes'],
                        'zero_match_scans': stats['zero_match_scans'],
                        'max_implied_keyframe_speed_mps': round(stats['raw_keyframe_max_implied_speed_mps'], 2),
                        'diagnostic_flag': stats['quality_label'] or 'None; accuracy not established',
                        'config': name + '/config.yaml', 'image': 'images/' + name + '.png',
                        'cloud': 'point_clouds/' + name + '.pcd',
                        'cloud_sha256': stats['source_pcd_sha256']})
    (batch / 'results.json').write_text(json.dumps({'source_commit': metadata['commit'],
        'binary_sha256': metadata['binary_sha256'], 'results': results}, indent=2) + '\n')
    with (batch / 'results.csv').open('w') as f:
        writer = csv.DictWriter(f, fieldnames=list(results[0]))
        writer.writeheader()
        writer.writerows(results)
    lines = ['# Seven dataset results', '', '![All seven saved maps](images/overview.png)', '',
             'All recordings used one executable, live visualization, loop closing on and the height prior off. '
             'M20 uses `configs/M20.yaml`; all five Lite3 sequences use `configs/Lite3.yaml`. '
             'Only the diagnostic output path changes between runs within each family.', '',
             '**Execution completion does not establish reconstruction accuracy.** '
             'Flags describe zero-match scans and large raw keyframe displacement/time; '
             'these speeds are SLAM estimates, not measured robot velocities. '
             'See each image render JSON for the exact diagnostic rules. All full maps, including divergent output, are retained.', '',
             '| Dataset | Execution | Seconds | Saved points | Zero-match scans | Max implied speed (m/s) | Diagnostic flag | Files |',
             '| --- | --- | ---: | ---: | ---: | ---: | --- | --- |']
    for r in results:
        lines.append(f"| {r['dataset']} | {r['execution']} | {r['elapsed_seconds']:.2f} | {r['points']:,} | {r['zero_match_scans']} | {r['max_implied_keyframe_speed_mps']:.2f} | {r['diagnostic_flag']} | [PNG]({r['image']}) · [PCD]({r['cloud']}) · [config]({r['config']}) |")
    lines += ['', f"Source revision: `{metadata['commit']}` plus archived `source.patch` and configs.", '',
              f"Executable SHA-256: `{metadata['binary_sha256']}`.", '',
              'The complete runtime libraries, executed configurations, bag hashes, logs, trajectories and map hashes are archived in this directory. '
              'Original PCDs are unchanged. Images use display voxels and one rigid floor/camera orientation; no height crop or flattening.', '']
    (batch / 'results.md').write_text('\n'.join(lines))


def overview(batch, destination):
    """Seven map panels only: Library F at left, six maps in a 2 x 3 grid."""
    fig = plt.figure(figsize=(30, 12), facecolor='white')
    panels = [[.005, .015, .365, .97]]
    for i in range(6):
        row, col = divmod(i, 3)
        panels.append([.38 + col * .205, .51 if row == 0 else .015, .2, .475])
    for name, bounds in zip(ORDER, panels):
        stats = json.loads((batch / 'images' / (name + '-render.json')).read_text())
        points = np.load(batch / 'images' / (name + '-display.npz'))['xyz']
        ax = fig.add_axes(bounds)
        draw_cloud(ax, points, stats, point_size=.35, bar=False)
    destination.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(destination, dpi=200, facecolor='white')
    plt.close(fig)
    copy = batch / 'images/overview.png'
    if destination.resolve() != copy.resolve():
        shutil.copy2(destination, copy)
    (batch / 'images/overview-render.json').write_text(json.dumps({'datasets': ORDER, 'size_pixels': [6000, 2400],
        'layout': 'Library F at left; Office, Building 1, Building 2 across upper right; Building 3, Grass 2, Road 1 across lower right',
        'panel_bounds': panels, 'text': False, 'scale_bars': False, 'background': 'white',
        'projection': 'orthographic', 'elevation_degrees': 65, 'per_map_autofit': True,
        'height_crop': None, 'color_map': 'turbo', 'occlusion': 'Nearest point per projected pixel at 200 dpi',
        'source_metadata': [n + '-render.json' for n in ORDER]}, indent=2) + '\n')
    write_results(batch)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('batch', type=Path)
    parser.add_argument('--dataset', action='append', choices=ORDER)
    parser.add_argument('--voxel', type=float, default=.2)
    parser.add_argument('--overview', type=Path, help='Create overview using all seven cached renders.')
    args = parser.parse_args()
    if args.dataset or not args.overview:
        for name in args.dataset or ORDER:
            points, stats = prepare(args.batch, name, args.voxel)
            standalone(args.batch, name, points, stats)
            print(name, stats['points_full'], 'points; rendered', flush=True)
    if args.overview:
        overview(args.batch, args.overview)
        print('Overview:', args.overview, flush=True)


if __name__ == '__main__':
    main()
