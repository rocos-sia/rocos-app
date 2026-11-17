#!/usr/bin/env python3
"""Offline dual-arm configuration checks; never connects to hardware."""
import copy
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

binary = Path(sys.argv[1]).resolve()
source = Path(__file__).resolve().parents[2]
with tempfile.TemporaryDirectory(prefix='rocos-dualarm-config-') as name:
    root = Path(name)
    def write(path, data):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(data))
    for i, side in enumerate(('left', 'right')):
        folder = root / side
        folder.mkdir()
        shutil.copy(source / 'config/robots/talon/robot.urdf', folder / 'model.urdf')
        write(folder / 'robot.json', {
            'schema_version': 1, 'name': 'talon', 'package_version': '1.0.0',
            'urdf': 'model.urdf', 'base_link': 'base_link', 'tip_link': 'link_7',
            'ecat_id': i, 'hardware_config': str(source / 'config/robots/talon/hardware.yaml'),
            'master_type': 'mujoco', 'master_config': 'master.json'})
        write(folder / 'master.json', {'master': {'id': i, 'cycle': 1000, 'state': 'op'}})
        write(folder / 'device.json', {'schema_version': 1, 'id': f'offline_{side}',
              'expected_robot': {'name': 'talon', 'package_version': '1.0.0'}})
        write(folder / 'app.json', {'schema_version': 1, 'robot_config': 'robot.json',
              'device_config': 'device.json', 'tool_library': 'tools',
              'scripts': str(source / 'scripts'), 'web_root': str(source / 'web')})
    base = {'schema_version': 1, 'arms': {'left': {'config': 'left/app.json'},
            'right': {'config': 'right/app.json'}}, 'logging': {'directory': 'logs'}}
    cases = 0
    def run(data, expected, message=''):
        global cases
        write(root / 'entry.json', data)
        result = subprocess.run([str(binary), '--config', str(root / 'entry.json'), '--check_config'],
                                cwd='/', capture_output=True, text=True, timeout=10)
        output = result.stdout + result.stderr
        assert (result.returncode == 0) == expected, output
        assert not message or message in output, output
        cases += 1
    run(base, True)
    run(dict(base, workcell={'urdf': 'left/model.urdf', 'asset_root': 'left'}),
        False, 'workcell URDF must match both arm models')
    run(dict(base, workcell={'urdf': 'left/model.urdf', 'asset_root': 'left', 'typo': 1}),
        False, 'Unknown workcell field')
    for bad in ({}, [], {'schema_version': 9}, dict(base, typo=1), dict(base, http_port=0),
                dict(base, http_control_ttl_seconds=0), dict(base, diagnostics={'port': 8080}),
                dict(base, diagnostics={'port': 0}), dict(base, arms={'left': base['arms']['left']})):
        run(bad, False)
    data = copy.deepcopy(base)
    data['arms']['right']['config'] = 'left/app.json'
    run(data, False, 'different ecat_id')
    robot_path = root / 'right/robot.json'
    robot = json.loads(robot_path.read_text())
    for key, value, expected in (('urdf', '/not/a/model.urdf', 'resource'),
                                ('tip_link', 'missing_link', '')):
        altered = dict(robot, **{key: value})
        write(robot_path, altered)
        run(base, False, expected)
    write(robot_path, robot)
    device_path = root / 'right/device.json'
    device = json.loads(device_path.read_text())
    write(device_path, dict(device, id='offline_left'))
    run(base, False, 'different device IDs')
    write(device_path, device)
    child_path = root / 'right/app.json'
    child = json.loads(child_path.read_text())
    write(child_path, dict(child, tool_library='../left/tools'))
    run(base, False, 'different tool libraries')
    write(child_path, child)
    write(robot_path, dict(robot, urdf='../left/model.urdf'))
    run(base, False, 'same joint')
    write(robot_path, robot)
    write(child_path, dict(child, device_config='device2.json'))
    write(root / 'right/device2.json', dict(device, id='offline_right_2'))
    # Distinct devices sharing a directory would overwrite frames/payload files.
    left_child_path = root / 'left/app.json'
    left_child = json.loads(left_child_path.read_text())
    write(left_child_path, dict(left_child, device_config='../right/device.json'))
    run(base, False, 'different frame paths')
    print(f'PASS: {cases} offline configuration cases')
