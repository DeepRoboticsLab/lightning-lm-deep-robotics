#!/usr/bin/env python3
"""Replay the seven local bags sequentially with one fixed preset per sensor family."""
import argparse
import datetime as dt
import hashlib
import json
import os
from pathlib import Path
import re
import shlex
import shutil
import sqlite3
import subprocess
import time

import yaml

REPO = Path(__file__).resolve().parents[1]
DATASETS = [
    ('office', 'M20', 'm20_lidar_data/office/lidar_data_bag_0.db3'),
    ('building3', 'Lite3', 'lite3_lidar_data/building3/building3_0.db3'),
    ('building2', 'Lite3', 'lite3_lidar_data/building2/building2_0.db3'),
    ('grass2', 'Lite3', 'lite3_lidar_data/grass2/grass2_0.db3'),
    ('building1', 'Lite3', 'lite3_lidar_data/building1/building1_0.db3'),
    ('road1', 'Lite3', 'lite3_lidar_data/road1/road1_0.db3'),
    ('libraryf', 'M20', 'm20_lidar_data/libraryf/libraryf_0.db3'),
]
CONFIGS = {'M20': 'config/libraryf_march18_3d.yaml', 'Lite3': 'config/lite3_livox_3d.yaml'}


def sha(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def save(path, value):
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, indent=2) + '\n')
    temporary.replace(path)


def git(*args):
    return subprocess.check_output(['git', '-C', str(REPO), *args], text=True).strip()


def main():
    global REPO, CONFIGS
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=REPO, help='Source checkout containing the compiled bin/run_slam_offline.')
    parser.add_argument('--build', type=Path, help='CMake build directory; defaults to SOURCE/libraryf-build.')
    parser.add_argument('--m20-config', help='M20 preset, absolute or relative to SOURCE.')
    parser.add_argument('--lite3-config', help='Lite3 preset, absolute or relative to SOURCE.')
    parser.add_argument('--dataset', action='append', choices=[d[0] for d in DATASETS], help='Replay selected datasets only; default: all seven.')
    parser.add_argument('--data-root', type=Path, default=REPO.parent / 'dataset')
    parser.add_argument('--output', type=Path, help='New output directory; existing directories are never overwritten.')
    parser.add_argument('--no-ui', action='store_true', help='Disable live visualization; saved point clouds are still produced.')
    args = parser.parse_args()
    REPO = args.source.resolve()
    CONFIGS = {'M20': args.m20_config or CONFIGS['M20'], 'Lite3': args.lite3_config or CONFIGS['Lite3']}
    datasets = [d for d in DATASETS if not args.dataset or d[0] in args.dataset]
    build = args.build.resolve() if args.build else REPO / 'libraryf-build'
    binary = REPO / 'bin/run_slam_offline'
    if not binary.is_file() or not (build / 'src/liblightning.libs.so').is_file():
        parser.error('Build first with: bash scripts/build_libraryf.sh')
    if not (build / 'thirdparty/livox_ros_driver/liblivox_ros_driver2__rosidl_typesupport_fastrtps_cpp.so').is_file():
        parser.error('Missing Livox deserialization library. Rebuild with: bash scripts/build_libraryf.sh')
    for _, _, relative in datasets:
        if not (args.data_root / relative).is_file():
            parser.error(f'Missing bag: {args.data_root / relative}')
    now = dt.datetime.now(dt.timezone.utc)
    output = (args.output or REPO / 'outputs' / ('seven-datasets-' + now.strftime('%Y%m%dT%H%M%S%fZ'))).resolve()
    output.mkdir(parents=True, exist_ok=False)
    runtime = output / 'runtime'
    runtime.mkdir()
    shutil.copy2(binary, runtime / binary.name)
    for lib in sorted(build.rglob('*.so')):
        shutil.copy2(lib, runtime / lib.name)
    presets = output / 'configs'
    presets.mkdir()
    for family, relative in CONFIGS.items():
        shutil.copy2(REPO / relative, presets / (family + '.yaml'))
    (output / 'source.patch').write_text(git('diff', 'HEAD', '--binary') + '\n')
    for relative in git('ls-files', '--others', '--exclude-standard', '--', 'src', 'cmake', 'config', 'scripts').splitlines():
        source_file = REPO / relative
        if source_file.is_file():
            destination = output / 'untracked-source' / relative
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source_file, destination)
    cpus = sorted(os.sched_getaffinity(0))[:8]
    batch = {'created_utc': now.isoformat(), 'source': str(REPO), 'commit': git('rev-parse', 'HEAD'),
             'branch': git('branch', '--show-current'), 'build': str(build), 'ui': not args.no_ui,
             'binary_sha256': sha(runtime / binary.name), 'libraries': {p.name: sha(p) for p in runtime.glob('*.so')},
             'cpu_affinity': cpus, 'omp_threads': 4, 'configs': CONFIGS, 'runs': []}
    save(output / 'batch.json', batch)
    print(f'Batch output: {output}', flush=True)
    for name, family, relative in datasets:
        run = output / name
        run.mkdir()
        bag = (args.data_root / relative).resolve()
        metadata = yaml.safe_load((bag.parent / 'metadata.yaml').read_text())['rosbag2_bagfile_information']
        with sqlite3.connect('file:' + str(bag) + '?mode=ro', uri=True) as db:
            topics = db.execute('select name,type from topics').fetchall()
        required = [('/IMU', 'sensor_msgs/msg/Imu'), ('/LIDAR/POINTS', 'sensor_msgs/msg/PointCloud2')] if family == 'M20' else [('/livox/imu', 'sensor_msgs/msg/Imu'), ('/livox/lidar', 'livox_ros_driver2/msg/CustomMsg')]
        if not all(topic in topics for topic in required):
            raise RuntimeError(f'Unexpected sensor topics in {bag}: {topics}')
        cfg = yaml.safe_load((presets / (family + '.yaml')).read_text())
        cfg['system']['with_ui'] = not args.no_ui
        cfg['diagnostics'] = {'output_directory': str(run / 'diagnostics')}
        (run / 'config.yaml').write_text(yaml.safe_dump(cfg, sort_keys=False))
        env = {'HOME': str(Path.home()), 'USER': os.environ.get('USER', 'ubuntu'),
               'PATH': '/usr/local/bin:/usr/bin:/bin', 'OMP_NUM_THREADS': '4',
               'OPENBLAS_NUM_THREADS': '1', 'MKL_NUM_THREADS': '1', 'ROS_LOCALHOST_ONLY': '1',
               'ROS_DOMAIN_ID': '87', 'ROS_LOG_DIR': str(run / 'ros-logs'), 'LD_LIBRARY_PATH': str(runtime)}
        if not args.no_ui:
            env.update(DISPLAY=os.environ.get('DISPLAY', ':1'),
                       XAUTHORITY=os.environ.get('XAUTHORITY', f'/run/user/{os.getuid()}/gdm/Xauthority'),
                       XDG_RUNTIME_DIR=os.environ.get('XDG_RUNTIME_DIR', f'/run/user/{os.getuid()}'),
                       PANGOLIN_WINDOW_URI='x11://')
        shell = 'source /opt/ros/humble/setup.bash; exec ' + shlex.join([
            'taskset', '-c', ','.join(map(str, cpus)), str(runtime / binary.name),
            '--input_bag', str(bag), '--config', str(run / 'config.yaml'), '--log_dir', str(run)])
        command = ['/usr/bin/time', '-v', '-o', str(run / 'resources.txt'), 'bash', '--noprofile', '--norc', '-c', shell]
        manifest = {'dataset': name, 'family': family, 'status': 'running', 'source_commit': batch['commit'],
                    'binary_sha256': batch['binary_sha256'], 'config_source': CONFIGS[family],
                    'config_sha256': sha(run / 'config.yaml'), 'environment': env, 'command': command,
                    'cpu_affinity': cpus, 'bag': {'path': str(bag), 'sha256': sha(bag), 'size_bytes': bag.stat().st_size,
                    'duration_seconds': metadata['duration']['nanoseconds'] / 1e9, 'topics': topics,
                    'message_counts': {x['topic_metadata']['name']: x['message_count'] for x in metadata['topics_with_message_count']}},
                    'start_utc': dt.datetime.now(dt.timezone.utc).isoformat()}
        save(run / 'manifest.json', manifest)
        batch['runs'].append({'dataset': name, 'family': family, 'directory': name, 'status': 'running'})
        save(output / 'batch.json', batch)
        print(f'Running {family}/{name} with {CONFIGS[family]}', flush=True)
        start = time.monotonic()
        with (run / 'run.log').open('w') as log:
            result = subprocess.run(command, cwd=run, env=env, stdout=log, stderr=subprocess.STDOUT)
        log = (run / 'run.log').read_text(errors='replace')
        cloud = run / 'data/new_map/global.pcd'
        complete = result.returncode == 0 and 'map saved' in log and ' finished.' in log and cloud.is_file()
        manifest.update(status='complete' if complete else 'failed', exit_status=result.returncode,
                        elapsed_seconds=time.monotonic() - start, end_utc=dt.datetime.now(dt.timezone.utc).isoformat(),
                        counts={'mapping_scans': log.count('[ mapping ]'), 'keyframes': log.count('LIO: create kf'),
                                'loop_optimizations': log.count('optimize finished, loops:'),
                                'lidar_reversed': log.count('lidar loop back, dt:') or log.count('lidar loop back, clear buffer'),
                                'imu_reversed': log.count('imu loop back'), 'sync_failures': log.count('sync package failed'),
                                'empty_scans': log.count('No point, skip')},
                        output_sha256={str(p.relative_to(run)): sha(p) for d in [run / 'data', run / 'diagnostics'] if d.exists() for p in d.rglob('*') if p.is_file()})
        match = re.search(r'Maximum resident set size \(kbytes\): (\d+)', (run / 'resources.txt').read_text())
        manifest['peak_rss_kib'] = int(match[1]) if match else None
        save(run / 'manifest.json', manifest)
        batch['runs'][-1]['status'] = manifest['status']
        save(output / 'batch.json', batch)
        print(f"{name}: {manifest['status']}, {manifest['elapsed_seconds']:.1f}s, {manifest['counts']}", flush=True)
    batch['completed_utc'] = dt.datetime.now(dt.timezone.utc).isoformat()
    save(output / 'batch.json', batch)
    print(f'All results: {output}', flush=True)
    return 0 if all(r['status'] == 'complete' for r in batch['runs']) else 1


if __name__ == '__main__':
    raise SystemExit(main())
