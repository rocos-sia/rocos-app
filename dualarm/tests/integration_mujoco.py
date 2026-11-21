#!/usr/bin/env python3
"""Opt-in, real two-Robot integration against the sibling dual-arm MuJoCo scene."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import json
import math
import os
from pathlib import Path
import socket
import struct
import subprocess
import time
import urllib.error
import urllib.request
from extended_checks import check_models, check_frames, check_motion
from calibration_checks import check_calibration

ROOT = Path(__file__).resolve().parents[2]
OPENER = urllib.request.build_opener(urllib.request.ProxyHandler({}))


def ipc(master_ids):
    names = []
    for i in master_ids:
        for pattern in (f'ecm_v2_{i}', f'pd_input_v2_{i}', f'pd_output_v2_{i}', f'sem.sync_v2_{i}_*'):
            names.extend(Path('/dev/shm').glob(pattern))
    return names


def wait_for(fn, timeout=10):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        value = fn()
        if value:
            return value
        time.sleep(0.025)
    raise TimeoutError(f'Condition did not pass within {timeout}s')


def call(path, body=None, headers=None, expected=0):
    request = urllib.request.Request('http://127.0.0.1:8080' + path,
        data=None if body is None else json.dumps(body).encode(),
        headers={'Content-Type': 'application/json', **(headers or {})})
    with OPENER.open(request, timeout=4) as response:
        value = json.load(response)
    assert value['code'] == expected, (path, value)
    assert value['success'] == (expected == 0), (path, value)
    return value.get('data')


class Arm:
    def __init__(self, side):
        self.side = side
        self.prefix = f'/api/arms/{side}/'
        self.headers = {}

    def request(self, path, body=None, expected=0):
        return call(self.prefix + path, body, self.headers, expected)

    def acquire(self):
        data = self.request('control/acquire', {'client_id': 'dualarm-integration', 'client_name': 'test'})
        self.headers = {'X-Rocos-Control-Token': data['token'], 'X-Rocos-Client-Id': data['client_id']}

    def state(self):
        data = self.request('robot/state')
        assert data['robot_state'] != 'ERROR_STATE', (self.side, data)
        assert [j['name'] for j in data['joint_states']] == [f'{self.side.title()}_Joint{i}' for i in range(1, 8)]
        assert all(math.isfinite(j[k]) for j in data['joint_states'] for k in ('position', 'velocity', 'torque', 'load_torque'))
        return data

    def move(self, target, velocity=0.1):
        return self.request('robot/movej', {'joints': target, 'velocity': velocity, 'acceleration': 0.3, 'jerk': 5})


def positions(state):
    return [j['position'] for j in state['joint_states']]


def error(actual, target):
    return max(abs(a - b) for a, b in zip(actual, target))


def settled(arm, target):
    state = arm.state()
    if not state['motion_busy'] and error(positions(state), target) < 0.003 and max(abs(j['velocity']) for j in state['joint_states']) < 0.01:
        return state
    return False


def wait_enabled_stable(arm):
    # Disabled simulated joints can drift; enable acceptance is not settling.
    stable_since = None
    def stable():
        nonlocal stable_since
        state = arm.state()
        if (state['is_enabled'] and state['robot_state'] == 'STOPPED'
                and max(abs(j['velocity']) for j in state['joint_states']) < 0.005):
            if stable_since is None:
                stable_since = time.monotonic()
            return time.monotonic() - stable_since >= 0.2
        stable_since = None
        return False
    wait_for(stable, 5)


def check_rci(left, right, client, report, output):
    start = right.prefix + 'robot/rci/start'
    enable = right.prefix + 'robot/rci/enable'
    call(start, {}, expected=-1018)
    call(enable, {}, expected=3006)
    call(enable, {}, left.headers, expected=3007)
    right.request('robot/rci/enable', {}, expected=-2212)
    for arm in (left, right):
        arm.request('robot/enable', {})
        wait_enabled_stable(arm)

    right.request('robot/rci/enable', {})
    call(start, {'port': 0}, expected=1001)
    call(start, {'mode': 'velocity'}, expected=1001)
    for action in ('movej', 'movel', 'jog/joint'):
        right.request(f'robot/{action}', {}, expected=-2215)
    call(right.prefix + 'robot/rci/disable', {}, expected=3006)
    right.request('robot/rci/disable', {})
    for arm in (left, right):
        arm.request('control/release', {})
    report['checks'].append('per-arm RCI credentials, startup validation and motion exclusion')

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as left_port, \
            socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as right_port:
        left_port.bind(('127.0.0.1', 0))
        right_port.bind(('127.0.0.1', 0))
        ports = [left_port.getsockname()[1], right_port.getsockname()[1]]
    sdk = subprocess.run([str(client), *map(str, ports)], text=True,
                         capture_output=True, timeout=14, check=False)
    (output / 'librocos.log').write_text(sdk.stdout + sdk.stderr)
    assert sdk.returncode == 0, sdk.stderr
    report['concurrent_udp'] = json.loads(sdk.stdout)
    report['checks'] += [
        'librocos right-arm UDP motion leaves the enabled left arm at its hold position',
        'librocos sessions move both arms concurrently on separate UDP ports',
        'right UDP loss faults its RCI session while the left session keeps applying commands',
        'librocos stop returns to RCI and disable returns to STOPPED',
    ]
    right.acquire()
    right.request('robot/reset', {})
    right.request('control/release', {})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--app', type=Path, default=ROOT / 'build/dualarm/bin/rocosDualArmMain')
    parser.add_argument('--sim', type=Path, default=ROOT.parent / 'rocos-mujoco/build/headless/bin/rocos_mujoco_sim')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--tool-packages', action='store_true',
                        help='Exercise configured package calibration with six valid simulated poses')
    parser.add_argument('--basic-motion', action='store_true',
                        help='Focus on independent MoveJ, control rights, calibration and logging; skip the extended IK/jog suite')
    parser.add_argument('--shoulder', action='store_true',
                        help='Use the opt-in shoulder-v1 bases, preserving seven axes')
    parser.add_argument('--rci-only', action='store_true',
                        help='Exercise concurrent independent UDP RCI sessions on isolated simulator masters')
    args = parser.parse_args()
    if args.rci_only and (args.tool_packages or args.basic_motion):
        parser.error('--rci-only cannot be combined with other motion suites')
    suffix = '_shoulder' if args.shoulder else ''
    urdf_name = f'dualarm{suffix}.urdf'
    sim_duration = 18 if args.rci_only else (120 if args.tool_packages else 65)
    master_ids = (os.getpid() * 2, os.getpid() * 2 + 1) if args.rci_only else (0, 1)
    args.app = args.app.resolve()
    args.sim = args.sim.resolve()
    args.output = (args.output or ROOT / 'build/dualarm' /
                   ('integration-rci' if args.rci_only else 'integration')).resolve()
    if ipc(master_ids):
        raise RuntimeError(f'Master IPC already exists: {ipc(master_ids)}')
    for port in (8080, 8082):
        with socket.socket() as sock:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind(('127.0.0.1', port))
    args.output.mkdir(parents=True, exist_ok=True)
    logs, app, sim = [], None, None
    report = {'passed': False, 'checks': [], 'app_binary': str(args.app), 'motion_suite': 'basic' if args.basic_motion else 'extended', 'coordinate_profile': 'shoulder-v1' if args.shoulder else 'legacy'}
    if args.rci_only:
        report['motion_suite'] = 'rci'
        report['master_ids'] = list(master_ids)
    arms = [Arm('left'), Arm('right')]

    def start(name, command):
        log = (args.output / f'{name}.log').open('w')
        logs.append(log)
        return subprocess.Popen([str(x) for x in command], cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)

    try:
        subprocess.run([str(args.app), '--config', str(ROOT / f'dualarm/config/app{suffix}.yaml'), '--check_config'],
                       check=True, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, timeout=15)
        model = ROOT.parent / 'rocos-mujoco/config/dualarm'
        config = args.output / 'simulation.json'
        config.write_text(json.dumps({'model': str(model / f'mjcf/scene{suffix}.xml'), 'masters': [
            {'ecat_id': i, 'hardware_config': str(model / f'hardware_{side}.yaml')}
            for i, side in zip(master_ids, ('left', 'right'))], 'cycle_time_us': 1000,
            'duration_sec': sim_duration, 'visualize': False}))
        # All persistence tests write to owned fixtures, never source device files.
        for master_id, side in zip(master_ids, ('left', 'right')):
            device_dir = args.output / side
            (device_dir / 'identification').mkdir(parents=True, exist_ok=True)
            for filename in ('frames.yaml', 'identification/payload.yaml'):
                (device_dir / filename).unlink(missing_ok=True)
            device = {'schema_version': 1, 'id': f'dualarm_integration_{side}',
                      'expected_robot': {'name': f'dualarm_{side}' + ('_shoulder_v1' if args.shoulder else ''), 'package_version': '1.0.0'}}
            if args.tool_packages:
                source = device_dir / 'package-source'
                source.mkdir(exist_ok=True)
                (source / 'tool.yaml').write_text('schema_version: 1\nname: calibration_package\n'
                    'package_version: 1.0.0\ntcp: {position: [0, 0, 0.05], orientation: [0, 0, 0, 1]}\n')
                device['tool_config'] = str(source / 'tool.yaml')
                device['tool_install'] = {'parent_link': f'{side.title()}_Link7',
                    'position': [0.02, 0.01, 0.03], 'orientation': [0, 0, math.sin(0.15), math.cos(0.15)]}
            (device_dir / 'device.json').write_text(json.dumps(device))
            child = {'schema_version': 1, 'robot_config': str(model / f'app/robot_{side}{suffix}.yaml'),
                     'device_config': str(device_dir / 'device.json'), 'tool_library': str(device_dir / 'tools'),
                     'scripts': str(ROOT / 'scripts'), 'web_root': str(ROOT / 'web')}
            if args.rci_only:
                override = device_dir / 'sim-master.json'
                override.write_text(json.dumps({'robot': {
                    'ecat_id': master_id, 'master': {'id': master_id}}}))
                child['user_config'] = str(override)
            (args.output / f'{side}.json').write_text(json.dumps(child))
        app_config = args.output / 'app.json'
        app_config.write_text(json.dumps({'schema_version': 1,
            'workcell': {'urdf': str(model / 'urdf' / urdf_name), 'asset_root': str(model)},
            'arms': {side: {'config': f'{side}.json'} for side in ('left', 'right')},
            'logging': {'directory': str(args.output / 'logs')}}))
        sim = start('simulator', [args.sim, '--config', config])
        sim_deadline = time.monotonic() + sim_duration + 10
        def sim_ready():
            assert sim.poll() is None, 'Simulator exited during startup'
            return all(Path(f'/dev/shm/ecm_v2_{i}').exists() for i in master_ids)
        wait_for(sim_ready)
        time.sleep(0.1)
        app = start('app', [args.app, '--config', app_config])
        def ready():
            assert app.poll() is None, f'App exited during startup: {app.returncode}'
            try:
                return call('/api/arms')
            except urllib.error.URLError:
                return False
        listing = wait_for(ready, 15)
        assert [a['ecat_id'] for a in listing['arms']] == list(master_ids)
        assert [a['joint_count'] for a in listing['arms']] == [7, 7]
        assert [a['base_link'] for a in listing['arms']] == (['left_base', 'right_base'] if args.shoulder else ['base_link', 'base_link'])
        report['pids'] = {'app': app.pid, 'simulator': sim.pid}
        report['checks'].append('one App exposes two independent 7-axis Robots')
        for arm in arms:
            assert not arm.state()['is_enabled']
            assert arm.request('robot/info')
            assert not arm.request('robot/move_status')['is_running']
            arm.acquire()
        left, right = arms
        if args.rci_only:
            assert listing['capabilities']['rci'] and not listing['capabilities']['synchronized_motion']
            check_rci(left, right, args.app.with_name('dualarm_rci_client'), report, args.output)
            app.terminate()
            assert app.wait(timeout=12) == 0, 'Orderly shutdown failed'
            report['passed'] = True
            return
        check_models(call, OPENER, model, report, urdf_name)
        check_frames(arms, call, OPENER, report)
        assert left.headers['X-Rocos-Control-Token'] != right.headers['X-Rocos-Control-Token']
        busy = left.request('control/acquire', {'client_id': 'another-client'}, 3008)
        assert busy['has_owner'] and 'token' not in busy
        assert right.request('control/status')['has_owner']
        call(right.prefix + 'robot/enable', {}, left.headers, 3007)
        call(left.prefix + 'robot/enable', {}, {}, 3006)
        call(left.prefix + 'robot/enable', {}, {**left.headers, 'X-Rocos-Client-Id': 'wrong-client'}, 3007)
        call(left.prefix + 'robot/enable', {}, {'X-Rocos-Control-Token': 'debug-token', 'X-Rocos-Client-Id': 'debug-client'}, 3007)
        call('/api/arms/wrong/robot/enable', {}, left.headers, 1001)
        left.request('robot/reset', {}, -2017)  # Reset is rejected outside ERROR_STATE.
        for path in ('/api/robot/enable', '/api/arms/left/robot/unsupported'):
            try:
                call(path, {}, left.headers)
                raise AssertionError(f'Unexpected route: {path}')
            except urllib.error.HTTPError as e:
                assert e.code == 404
                assert json.load(e)['success'] is False
        assert not left.state()['is_enabled'] and not right.state()['is_enabled']
        left.request('robot/enable', {})
        wait_for(lambda: left.state()['is_enabled'])
        assert not right.state()['is_enabled']
        right.request('robot/enable', {})
        wait_for(lambda: right.state()['is_enabled'])
        for arm in arms:
            wait_enabled_stable(arm)
        report['checks'].append('independent enable; wrong arm/token/debug credentials cannot actuate')
        check_calibration(arms, call, wait_for, report)
        if args.tool_packages:
            from package_calibration_checks import check_package_calibration
            check_package_calibration(arms, wait_for, report, args.output)
        left.request('robot/movej', {'joints': [0]}, 1003)
        left.request('robot/movej', {'joints': [0]*7, 'velocity': 'bad'}, 1001)
        left.request('robot/movej', {'joints': [0]*7, 'jerk': 0}, 1001)
        for moving, holding, target in ((left, right, [0.04, -0.03, 0.02, -0.04, 0.03, -0.02, 0.04]),
                                        (right, left, [-0.03, 0.04, -0.04, 0.02, -0.02, 0.03, -0.04])):
            baseline = positions(holding.state())
            maximum_drift = 0
            moving.move(target)
            wait_for(lambda: moving.state()['motion_busy'], 2)
            for action, body in (('pose', {'name': 'busy', 'index': 1}),
                                 ('run', {'session_id': 'busy'}),
                                 ('save', {'session_id': 'busy'})):
                moving.request('calibration/' + action, body, 4005)
            def arrived():
                nonlocal maximum_drift
                maximum_drift = max(maximum_drift, error(positions(holding.state()), baseline))
                assert maximum_drift < 0.005
                return settled(moving, target)
            final = wait_for(arrived)
            report['checks'].append({'independent_motion': moving.side, 'target_rad': target,
                'actual_rad': positions(final), 'max_error_rad': error(positions(final), target),
                'other_arm_max_drift_rad': maximum_drift})
        if not args.basic_motion:
            check_motion(arms, call, wait_for, positions, error, report)
        def pair(l_target, r_target, velocity):
            with ThreadPoolExecutor(max_workers=2) as pool:
                jobs = [pool.submit(a.move, t, velocity) for a, t in zip(arms, (l_target, r_target))]
                for job in jobs:
                    job.result()
            wait_for(lambda: all(a.state()['motion_busy'] for a in arms), 2)
        pair([0.12]*7, [-0.12]*7, 0.06)
        time.sleep(0.25)
        left.request('robot/pause', {})
        wait_for(lambda: left.state()['robot_state'] == 'PAUSED', 2)
        assert right.state()['motion_busy'], 'Pausing left paused right'
        left.request('robot/resume', {})
        wait_for(lambda: left.state()['robot_state'] == 'RUNNING', 2)
        call(right.prefix + 'robot/stop', {}, left.headers, 3007)
        left.request('robot/stop', {})
        wait_for(lambda: not left.state()['motion_busy'], 4)
        assert right.state()['motion_busy'], 'Stopping left stopped right'
        time.sleep(0.2)
        left_held = positions(left.state())
        wait_for(lambda: settled(right, [-0.12]*7))
        assert error(positions(left.state()), left_held) < 0.005
        report['checks'].append('concurrent requests; left pause/resume/Stop leaves right motion running')
        targets = ([0.02, -0.02, 0.03, -0.03, 0.02, -0.02, 0.03],
                   [-0.02, 0.03, -0.02, 0.02, -0.03, 0.02, -0.03])
        pair(*targets, 0.08)
        for arm, target in zip(arms, targets):
            final = wait_for(lambda: settled(arm, target))
            report['checks'].append({'concurrent_motion': arm.side,
                'actual_rad': positions(final), 'max_error_rad': error(positions(final), target)})
        left.request('robot/disable', {})
        wait_for(lambda: left.state()['robot_state'] == 'IDLE' and not left.state()['is_enabled'])
        assert right.state()['is_enabled']
        left.request('control/release', {})
        left.request('robot/enable', {}, 3007)
        right.request('control/heartbeat', {})
        left.acquire()
        left.request('robot/enable', {})
        wait_enabled_stable(left)
        report['checks'].append('independent disable/release; released token rejected; right lease survives')
        # Read-only diagnostics: shared stream, correct arm attribution, and no credentials.
        def diagnostic_events():
            with OPENER.open('http://127.0.0.1:8082/api/dualarm/logs?limit=500', timeout=4) as response:
                return json.load(response)['data']['events']
        events = wait_for(lambda: (v if any(e['operation'] and e['operation']['name'] == 'POST control/release' for e in v) else False) if (v := diagnostic_events()) else False)
        for side, index in [('left', 0), ('right', 1)]:
            scoped = [e for e in events if e['arm'] == side]
            assert scoped and all(e['ecat_id'] == index and e['device_id'] == f'dualarm_integration_{side}' for e in scoped)
            assert any(e['module'] == 'Robot' and 'Motion completed' in e['message'] for e in scoped)
            assert any(e['operation'] and e['operation']['name'] == 'POST robot/movej' and e['operation']['outcome'] == 'accepted' for e in scoped)
            assert any(e['operation'] and e['code'] == 3007 and e['operation']['outcome'] == 'rejected' for e in scoped)
        assert any(e['arm'] == 'process' and e['code'] == 1001 and e['operation'] for e in events)
        assert any(e['operation'] and e['operation']['name'] == 'POST calibration/pose' for e in events)
        assert not any(e['operation'] and e['operation']['name'] == 'POST control/heartbeat' and e['operation']['outcome'] == 'accepted' for e in events)
        assert all(100 <= e["operation"]["http_status"] <= 599 for e in events if e["operation"])
        serialized = json.dumps(events)
        for arm in arms:
            assert arm.headers['X-Rocos-Control-Token'] not in serialized
        with OPENER.open('http://127.0.0.1:8082/api/logs?limit=500', timeout=4) as response:
            alerts = json.load(response)['data']['events']
        assert all(len(e) == 6 and e['level'] in ('warn', 'error', 'critical') for e in alerts)
        with OPENER.open('http://127.0.0.1:8082/api/logs/file/latest', timeout=4) as response:
            file_text = response.read().decode()
        assert 'arm=left ecat_id=0' in file_text and 'arm=right ecat_id=1' in file_text
        assert 'request_id=' in file_text
        for arm in arms:
            assert arm.headers['X-Rocos-Control-Token'] not in file_text
        (args.output / 'diagnostics-events.json').write_text(json.dumps(events, indent=2))
        report['checks'].append('INFO operations and Robot completion logs carry correct arm/device/master; rejected credentials audited without secrets; legacy six-field alerts unchanged; scoped file snapshot')
        pair([0.10]*7, [-0.10]*7, 0.05)
        app.terminate()
        assert app.wait(timeout=12) == 0, 'Orderly shutdown failed'
        time.sleep(0.1)
        # Known dualarm hardware layout: seven 30-byte input PDOs, status first.
        for i in (0, 1):
            data = Path(f'/dev/shm/pd_input_v2_{i}').read_bytes()
            for joint in range(7):
                status = struct.unpack_from('<H', data, 30*joint)[0]
                assert status & 0x6f != 0x27, (i, joint, status)
        report['checks'].append('SIGTERM during both motions exits cleanly and disables all 14 drives')
        report['passed'] = True
    except Exception as failure:
        report['failure'] = str(failure)
        print(f'Integration failed: {failure}', flush=True)
        raise
    finally:
        cleanup = []
        if app is not None and app.poll() is None:
            app.terminate()
            try:
                app.wait(timeout=12)
            except subprocess.TimeoutExpired:
                app.kill(); app.wait(); cleanup.append('App shutdown timeout')
        if sim is not None:
            print('Waiting for finite MuJoCo run to clean up IPC.', flush=True)
            while sim.poll() is None and time.monotonic() < sim_deadline:
                try:
                    sim.wait(timeout=1)
                except subprocess.TimeoutExpired:
                    pass
            if sim.poll() is None:
                sim.kill(); sim.wait(); cleanup.append('Simulator timeout')
            if sim.returncode != 0:
                cleanup.append(f'Simulator exit={sim.returncode}')
        for log in logs:
            log.close()
        if ipc(master_ids):
            cleanup.append(f'Remaining IPC: {ipc(master_ids)}')
        report['cleanup_errors'] = cleanup
        report['passed'] = report['passed'] and not cleanup
        (args.output / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
        if cleanup:
            raise RuntimeError('; '.join(cleanup))
        if report['passed']:
            print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
