"""Opt-in package calibration using actual MuJoCo feedback, never injected samples."""
import math
import time

import numpy as np
import yaml


def multiply(a, b):
    x, y, z, w = a
    X, Y, Z, W = b
    return np.array([w*X+x*W+y*Z-z*Y, w*Y-x*Z+y*W+z*X,
                     w*Z+x*Y-y*X+z*W, w*W-x*X-y*Y-z*Z])


def euler(x, y, z):
    return multiply(multiply([0, 0, math.sin(z/2), math.cos(z/2)],
                             [0, math.sin(y/2), 0, math.cos(y/2)]),
                    [math.sin(x/2), 0, 0, math.cos(x/2)])


def rotation(q):
    x, y, z, w = np.array(q) / np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def pose(p, q):
    return {'position': dict(zip('xyz', map(float, p))),
            'orientation': dict(zip('xyzw', map(float, q)))}


def arrays(value):
    return (np.array([value['position'][k] for k in 'xyz']),
            np.array([value['orientation'][k] for k in 'xyzw']))


def stationary(arm):
    state = arm.state()
    return not state['motion_busy'] and max(abs(j['velocity']) for j in state['joint_states']) < 0.0008


def check_package_calibration(arms, wait_for, report, output):
    name = 'calibration_package'
    for arm in arms:
        arm.request('robot/active_tool_frame', {'name': ''})
    for index, arm in enumerate(arms):
        other = arms[1-index]
        before = arm.request('robot/tool_frame?name=' + name)['package']
        other_before = other.request('robot/tool_frame?name=' + name)
        source = output / arm.side / 'package-source' / 'tool.yaml'
        source_before = source.read_bytes()
        initial = [0.2, -0.4, 0.2, -0.6, 0.2, 0.4, 0.1]
        arm.request('robot/movej', {'joints': initial, 'velocity': 0.35,
                                  'acceleration': 0.8, 'jerk': 5})
        time.sleep(0.1)
        wait_for(lambda: stationary(arm), 15)
        base_p, base_q = arrays(arm.state()['flange'])
        expected_p, expected_q = np.array([0.015, -0.01, 0.1]), euler(0.1, -0.15, 0.2)
        pivot = base_p + rotation(base_q) @ expected_p
        samples = []
        for delta in [(0.15, 0, 0), (0, 0.15, 0), (-0.12, -0.1, 0.08), (0, 0, 0)]:
            q = multiply(base_q, euler(*delta))
            samples.append(pose(pivot - rotation(q) @ expected_p, q))
        for offset in ([0.015, 0, 0], [0, 0, 0.015]):
            samples.append(pose(base_p + rotation(base_q) @ rotation(expected_q) @ offset, base_q))
        sid = ''
        for i, target in enumerate(samples, 1):
            arm.request('robot/movej_ik', {'pose': target, 'velocity': 0.2,
                                         'acceleration': 0.6, 'jerk': 5})
            time.sleep(0.1)
            wait_for(lambda: stationary(arm), 10)
            state = arm.request('calibration/pose', {'name': name, 'index': i, 'session_id': sid})
            sid = state['session_id']
            assert state['package_digest'] == before['digest']
            assert state['package_revision'] == before['revision']
        result = arm.request('calibration/run', {'session_id': sid, 'max_error': 0.002})
        solved_p, solved_q = arrays(result['result']['pose'])
        tcp_error = float(np.linalg.norm(solved_p - expected_p))
        assert tcp_error < 0.005, (arm.side, result)
        assert np.linalg.norm(rotation(solved_q) - rotation(expected_q)) < 0.02
        other.request('calibration/save', {'session_id': sid}, 4004)
        arm.request('calibration/save', {'session_id': sid})
        saved = arm.request('robot/tool_frame?name=' + name)
        package = saved['package']
        assert package['digest'] == before['digest'] and package['revision'] > before['revision']
        assert package['mount'] == before['mount']
        mount_p, mount_q = arrays(package['mount'])
        local_p, local_q = arrays(package['tcp'])
        assert np.linalg.norm(mount_p + rotation(mount_q) @ local_p - solved_p) < 1e-8
        assert np.linalg.norm(rotation(mount_q) @ rotation(local_q) - rotation(solved_q)) < 1e-8
        assert arm.request('robot/tool_frames')['active'] == ''
        assert other.request('robot/tool_frame?name=' + name) == other_before
        assert source.read_bytes() == source_before
        metadata = yaml.safe_load(package['calibration_yaml'])
        assert metadata['robot_id'] == 'dualarm_integration_' + arm.side
        assert metadata['package_digest'] == before['digest']
        assert metadata['timestamp'] > 0 and 'condition_number' in metadata
        current = arm.request('calibration/result?session_id=' + sid)
        assert current['package_revision'] == package['revision']
        arm.request('calibration/save', {'session_id': sid})
        repeated = arm.request('robot/tool_frame?name=' + name)['package']
        assert repeated['revision'] > package['revision']
        arm.request('robot/frames/load', {})
        restored = arm.request('robot/tool_frame?name=' + name)['package']
        restored_p, restored_q = arrays(restored['tcp'])
        repeated_p, repeated_q = arrays(repeated['tcp'])
        assert np.allclose(restored_p, repeated_p, atol=1e-10)
        assert np.allclose(rotation(restored_q), rotation(repeated_q), atol=1e-10)
        assert restored['calibration_yaml'] == repeated['calibration_yaml']
        arm.request('calibration/result?session_id=' + sid, expected=4004)
        # Loading installation state invalidates an existing capture session.
        stale = arm.request('calibration/pose', {'name': name, 'index': 1})['session_id']
        # A fresh load preserves the package but invalidates the arm-local session.
        arm.request('robot/frames/load', {})
        arm.request('calibration/run', {'session_id': stale}, 4004)
        arm.request('robot/movej', {'joints': [0]*7, 'velocity': 0.35, 'acceleration': 0.8, 'jerk': 5})
        time.sleep(0.1)
        wait_for(lambda: stationary(arm), 15)
        report['checks'].append({'package_calibration': arm.side, 'valid_real_samples': 6,
            'tcp_error_m': tcp_error, 'saved_local_tcp': True, 'reloaded': True,
            'other_arm_unchanged': True, 'source_package_unchanged': True})
