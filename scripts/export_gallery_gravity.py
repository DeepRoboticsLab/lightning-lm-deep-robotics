#!/usr/bin/env python3
"""Recover the initialized gravity direction for a single rigid display rotation."""
import argparse
import json
from pathlib import Path
import re
import sqlite3
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('batch', type=Path)
    args = parser.parse_args()
    for run in sorted(args.batch.iterdir()):
        if not (run / 'manifest.json').exists():
            continue
        manifest = json.loads((run / 'manifest.json').read_text())
        match = re.search(r'imu init done, bg:\s*(.*?), (?:ba|grav):', (run / 'run.log').read_text(errors='replace'))
        if not match:
            continue
        bias = np.fromstring(match[1], sep=' ')
        topic = '/IMU' if manifest['family'] == 'M20' else '/livox/imu'
        with sqlite3.connect('file:' + manifest['bag']['path'] + '?mode=ro', uri=True) as db:
            tid = db.execute('select id from topics where name=?', (topic,)).fetchone()[0]
            messages = [deserialize_message(row[0], Imu) for row in db.execute(
                'select data from messages where topic_id=? order by timestamp limit 160', (tid,))]
        gyro = np.array([[m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z] for m in messages])
        accel = np.array([[m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z] for m in messages])
        means = np.cumsum(gyro, axis=0) / np.arange(1, len(messages) + 1)[:, None]
        errors = np.linalg.norm(means - bias, axis=1)
        count = int(np.argmin(errors)) + 1
        if errors[count - 1] > 2e-6:
            raise RuntimeError(f'{run.name}: initial gyro mean did not match logged bias: {errors[count - 1]}')
        up = accel[:count].mean(axis=0)
        up /= np.linalg.norm(up)
        data = {'method': 'Match initial IMU prefix mean angular velocity to the logged initialized gyro bias; normalize the same prefix mean acceleration as IMUInit. Display-only rigid rotation; no trajectory-height constraint.',
                'samples': count, 'logged_bias': bias.tolist(), 'gyro_match_error': float(errors[count - 1]),
                'initial_up_direction': up.tolist(), 'initial_axis_tilt_deg': float(np.degrees(np.arccos(up[2])))}
        (run / 'initial-gravity.json').write_text(json.dumps(data, indent=2) + '\n')
        print(run.name, count, 'samples, axis tilt', round(data['initial_axis_tilt_deg'], 2), flush=True)


if __name__ == '__main__':
    main()
