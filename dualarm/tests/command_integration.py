#!/usr/bin/env python3
"""Command protocol regression on owned masters, ephemeral ports and fixtures."""
import argparse
import concurrent.futures
import fcntl
import json
import os
from pathlib import Path
import socket
import subprocess
import time
import urllib.request

ROOT = Path(__file__).resolve().parents[2]
MUJOCO = ROOT.parent / 'rocos-mujoco'


def port():
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        return sock.getsockname()[1]


def wait(predicate, timeout=15):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        try:
            value = predicate()
            if value:
                return value
        except (OSError, ValueError):
            pass
        time.sleep(.03)
    raise AssertionError('Timed out waiting for isolated fixture')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--browser', type=Path, action='append', default=[])
    args = parser.parse_args()
    output = ROOT / 'build/dualarm/command-regression'
    output.mkdir(parents=True, exist_ok=True)
    masters = [60000000 + os.getpid() * 10 + index for index in range(2)]
    http, diagnostics = port(), port()
    while http == diagnostics:
        diagnostics = port()
    processes, logs, credentials, checks = [], [], {}, []

    def write(name, data):
        path = output / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(data))
        return str(path)

    arms = {}
    for index, side in enumerate(('left', 'right')):
        master = write(f'{side}-master.json', {'master': {'id': masters[index], 'cycle': 1000, 'state': 'op'}})
        robot = write(f'{side}-robot.json', {'schema_version': 1, 'name': f'dualarm_{side}_shoulder_v1', 'package_version': '1.0.0', 'urdf': str(MUJOCO / 'config/dualarm/urdf/dualarm_shoulder.urdf'), 'base_link': f'{side}_base', 'tip_link': f'{side.title()}_Link7', 'ecat_id': masters[index], 'hardware_config': str(MUJOCO / f'config/dualarm/hardware_{side}.yaml'), 'master_type': 'mujoco', 'master_config': master})
        device = write(f'{side}/device.json', {'schema_version': 1, 'id': f'command_regression_{side}', 'expected_robot': {'name': f'dualarm_{side}_shoulder_v1', 'package_version': '1.0.0'}})
        arms[side] = {'config': write(f'{side}-app.json', {'schema_version': 1, 'robot_config': robot, 'device_config': device, 'tool_library': str(output / f'{side}-tools'), 'scripts': str(ROOT / 'scripts'), 'web_root': str(ROOT / 'web')})}
    config = write('app.json', {'schema_version': 1, 'http_host': '127.0.0.1', 'http_port': http, 'http_control_ttl_seconds': 300, 'workcell': {'urdf': str(MUJOCO / 'config/dualarm/urdf/dualarm_shoulder.urdf'), 'asset_root': str(MUJOCO / 'config/dualarm')}, 'diagnostics': {'host': '127.0.0.1', 'port': diagnostics}, 'logging': {'directory': str(output / 'logs')}, 'arms': arms})
    simulation = write('sim.json', {'model': str(MUJOCO / 'config/dualarm/mjcf/scene_shoulder.xml'), 'masters': [{'ecat_id': master, 'hardware_config': str(MUJOCO / f'config/dualarm/hardware_{side}.yaml')} for master, side in zip(masters, arms)], 'cycle_time_us': 1000, 'duration_sec': 300, 'visualize': False})

    def start(name, command, cwd=ROOT, env=None):
        log = (output / f'{name}.log').open('w')
        logs.append(log)
        process = subprocess.Popen(command, cwd=cwd, env=env, stdout=log, stderr=subprocess.STDOUT)
        processes.append(process)
        return process

    def request(side, operation, body=None, expected=0, headers=None):
        path = f'/api/arms/{side}/{operation}'
        req = urllib.request.Request(f'http://127.0.0.1:{http}{path}', data=None if body is None else json.dumps(body).encode(), headers={'Content-Type': 'application/json', **(credentials.get(side, {}) if headers is None else headers)})
        with urllib.request.urlopen(req, timeout=6) as response:
            payload = json.load(response)
        assert payload['code'] in ((0, 100) if expected == 0 else (expected,)), (operation, payload)
        assert payload['success'] == (expected == 0), (operation, payload)
        return payload['data']

    def state(side):
        return request(side, 'robot/state')

    def acquire(side):
        data = request(side, 'control/acquire', {'client_id': f'command-{side}', 'command_protocol': 1}, headers={})
        assert data['command_protocol'] == 1
        credentials[side] = {'X-Rocos-Control-Token': data['token'], 'X-Rocos-Client-Id': data['client_id']}
        return data

    def command(side, identifier, delta=.02):
        snapshot = state(side)
        goal = [joint['position'] for joint in snapshot['joint_states']]
        goal[1] += delta
        return {'instance_id': snapshot['instance_id'], 'motion_epoch': snapshot['motion_epoch'], 'command_id': identifier, 'joints': goal, 'velocity': .1, 'acceleration': .3, 'jerk': 5}

    def finish(side, identifier):
        outcome = request(side, f'robot/command_status?command_id={identifier}')['command']
        return outcome if outcome['status'] in ('finished', 'stopped', 'rejected', 'failed') else False

    try:
        start('sim', [str(MUJOCO / 'build/headless/bin/rocos_mujoco_sim'), '--config', simulation], MUJOCO)
        def masters_ready():
            return all((Path('/dev/shm') / f'ecm_v2_{master}').exists() and (Path('/dev/shm') / f'ecm_v2_{master}').read_bytes()[68:72] == b'\x01\0\0\0' for master in masters)
        wait(masters_ready)
        app = start('app', [str(ROOT / 'build/dualarm/bin/rocosDualArmMain'), '--config', config])
        wait(lambda: all(state(side)['robot_state'] == 'IDLE' for side in arms))
        first_instance = state('left')['instance_id']
        for side in arms:
            acquire(side)
            request(side, 'robot/enable', {})
        wait(lambda: all(state(side)['robot_state'] == 'STOPPED' for side in arms))

        delayed = command('left', 'delayed')
        before = [joint['position'] for joint in state('left')['joint_states']]
        stopped = request('left', 'robot/stop', {'instance_id': first_instance})
        assert stopped['motion_epoch'] > delayed['motion_epoch']
        request('left', 'robot/movej', delayed, 4101)
        after = [joint['position'] for joint in state('left')['joint_states']]
        assert max(abs(a-b) for a, b in zip(before, after)) < 3e-6
        checks.append('A stop acknowledged before delayed MoveJ delivery rejects the old epoch without movement')
        request('left', 'robot/movej', {'joints': before}, 1001)
        request('left', 'robot/movej', {**command('left', 'old-instance'), 'instance_id': 'previous-app'}, 4105)
        checks.append('Negotiated protocol rejects missing fences and old App instances')

        body = command('left', 'once')
        accepted = request('left', 'robot/movej', body)
        assert accepted['command']['command_id'] == 'once'
        assert wait(lambda: finish('left', 'once'))['status'] == 'finished'
        duplicate = request('left', 'robot/movej', body)
        assert duplicate['command']['status'] == 'finished'
        assert duplicate['command']['command_id'] == 'once'
        request('left', 'robot/movej', {**body, 'velocity': .2}, 4102)
        checks.append('Repeated command ID returns the recorded result; changed payload is rejected')
        request('left', 'robot/command_status?command_id=once', expected=3006, headers={})
        request('left', 'robot/command_status?command_id=once', expected=3007, headers=credentials['right'])
        request('right', 'robot/command_status?command_id=once', expected=4103)
        checks.append('Command results require this arm owner and cannot be read with the other arm token')

        left, right = command('left', 'left-moving', .06), command('right', 'right-moving', -.04)
        with concurrent.futures.ThreadPoolExecutor(2) as pool:
            futures = [pool.submit(request, side, 'robot/movej', value) for side, value in [('left', left), ('right', right)]]
            for future in futures:
                future.result()
        request('left', 'robot/stop', {'instance_id': first_instance})
        assert wait(lambda: finish('left', 'left-moving'))['status'] == 'stopped'
        assert wait(lambda: finish('right', 'right-moving'))['status'] == 'finished'
        checks.append('Stopping left preserves right arm epoch and independent completion')

        request('left', 'robot/tool_frame', {'name': 'probe', 'frame': {'position': {'x': 0, 'y': 0, 'z': .1}, 'orientation': {'x': 0, 'y': 0, 'z': 0, 'w': 1}}})
        calibration = None
        for index in range(1, 5):
            calibration = request('left', 'calibration/pose', {'name': 'probe', 'index': index, **({'session_id': calibration['session_id']} if calibration else {})})
        assert request('left', f"calibration/result?session_id={calibration['session_id']}")['captured'] == 4
        checks.append('Server retains four stationary calibration samples for page reload recovery')

        app.terminate()
        app.wait(timeout=10)
        start('app-restart', [str(ROOT / 'build/dualarm/bin/rocosDualArmMain'), '--config', config])
        wait(lambda: state('left')['instance_id'] != first_instance)
        request('left', 'control/heartbeat', {}, 3007)
        acquire('left')
        request('left', 'robot/command_status?command_id=once', expected=4103)
        request('left', f"calibration/result?session_id={calibration['session_id']}", expected=4004)
        request('left', 'control/release', {})
        checks.append('App restart changes instance, rejects old control tokens and expires old command/calibration handles')

        if args.browser:
            viz = ROOT.parent / 'rocos-viz'
            preview = port()
            environment = {**os.environ, 'DUALARM_APP_ORIGIN': f'http://127.0.0.1:{http}'}
            start('viz', ['node', 'node_modules/.bin/vite', 'preview', '--config', 'design/dualarm/v2/vite.config.mjs', '--host', '127.0.0.1', '--port', str(preview), '--strictPort'], viz, environment)
            def preview_ready():
                with urllib.request.urlopen(f'http://127.0.0.1:{preview}', timeout=1) as response:
                    return response.status == 200
            wait(preview_ready)
            for script in args.browser:
                subprocess.run(['node', str(script.resolve())], cwd=viz, env={**environment, 'DUALARM_PREVIEW_URL': f'http://127.0.0.1:{preview}', 'DUALARM_DIAGNOSTICS_ORIGIN': f'http://127.0.0.1:{diagnostics}'}, check=True, timeout=150)
                checks.append(f'Browser regression passed: {script.name}')
        report = {'passed': True, 'masters': masters, 'ports': [http, diagnostics], 'checks': checks}
        (output / 'result.json').write_text(json.dumps(report, indent=2))
        print(json.dumps(report, indent=2))
    finally:
        for process in reversed(processes):
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
        for log in logs:
            log.close()
        for master in masters:
            bus = Path('/dev/shm') / f'ecm_v2_{master}'
            if not bus.exists():
                continue
            with bus.open('rb') as lock:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                for pattern in (f'ecm_v2_{master}', f'pd_input_v2_{master}', f'pd_output_v2_{master}', f'sem.sync_v2_{master}_*'):
                    for path in Path('/dev/shm').glob(pattern):
                        path.unlink(missing_ok=True)


if __name__ == '__main__':
    main()
