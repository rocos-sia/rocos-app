"""Additional opt-in checks used only by the owned MuJoCo integration run."""
import json
import time
from pathlib import Path
import urllib.error
import urllib.parse
import urllib.request


def check_models(call, opener, model_root, report):
    data = call('/api/workcell/model')
    assert [a['arm'] for a in data['arms']] == ['left', 'right']
    assert len(set(sum([a['joint_names'] for a in data['arms']], []))) == 14
    assert data['root_link'] == 'base_link' and data['revision']
    base = 'http://127.0.0.1:8080'
    with opener.open(base + data['urdf_url']) as response:
        xml, etag = response.read(), response.headers['ETag']
        assert 'xml' in response.headers['Content-Type']
    assert xml == (model_root / 'urdf/dualarm.urdf').read_bytes()
    for arm in ('left', 'right'):
        with opener.open(base + f'/api/arms/{arm}/robot/urdf') as response:
            assert response.read() == xml
    for asset in data['assets']:
        with opener.open(base + data['mesh_url'] + '?' + urllib.parse.urlencode({'path': asset['path']})) as response:
            content = response.read()
        assert len(content) == asset['bytes']
        assert content == (model_root / 'urdf' / asset['path']).resolve().read_bytes()
    for resource in ('/etc/passwd', '../../hardware_left.yaml', '../hardware_left.yaml', 'base_link.STL'):
        try:
            opener.open(base + data['mesh_url'] + '?' + urllib.parse.urlencode({'path': resource}))
            raise AssertionError('Unlisted model resource was served')
        except urllib.error.HTTPError as error:
            assert error.code == 404
    try:
        opener.open(urllib.request.Request(base + data['urdf_url'], headers={'If-None-Match': etag}))
        raise AssertionError('Model cache was not revalidated')
    except urllib.error.HTTPError as error:
        assert error.code == 304
    report['checks'].append({'model_download': True, 'assets_verified': len(data['assets']),
                             'revision': data['revision'], 'unlisted_resources_rejected': True})


def check_frames(arms, call, opener, report):
    left, right = arms
    identity = {'position': {'x': 0, 'y': 0, 'z': 0},
                'orientation': {'x': 0, 'y': 0, 'z': 0, 'w': 1}}
    for arm in arms:
        assert arm.request('robot/enabled')['enabled'] is False
        assert 'pose' in arm.request('robot/tcp')
    right_before = right.request('robot/tool_frames')
    for kind in ('tool', 'object'):
        body = {'name': 'integration_frame', 'frame': identity}
        call(right.prefix + f'robot/{kind}_frame', body, left.headers, 3007)
        left.request(f'robot/{kind}_frame', body)
        left.request(f'robot/active_{kind}_frame', {'name': 'integration_frame'})
        assert left.request(f'robot/{kind}_frame?name=integration_frame')['frame'] == identity
        assert 'integration_frame' not in right.request(f'robot/{kind}_frames')['names']
    assert right.request('robot/tool_frames') == right_before
    saved = left.request('robot/frames/save', {})
    left.request('robot/frames/load', {'path': saved['path']})
    left.request('robot/frames/save', {'path': right.request('robot/tool_frames')['file_path']}, 1001)
    left.request('robot/load_params', {'mass': 0, 'com': [0, 0, 0]})
    assert left.request('robot/load_params')['valid']
    left.request('robot/load_params/save', {})
    left.request('robot/load_params', {'mass': -1}, -2902)
    for kind in ('tool', 'object'):
        url = 'http://127.0.0.1:8080' + left.prefix + f'robot/{kind}_frame?name=integration_frame'
        request = urllib.request.Request(url, method='DELETE', headers=left.headers)
        with opener.open(request) as response:
            assert json.load(response)['success']
        assert 'integration_frame' not in left.request(f'robot/{kind}_frames')['names']
    report['checks'].append('per-arm TCP, frame CRUD/activation/persistence and payload; cross-arm writes rejected')


def check_motion(arms, call, wait_for, positions, error, report):
    left, right = arms
    for arm in arms:
        arm.request('robot/jog/joint', {'joints': [0]}, 1003)
        arm.request('robot/jog/cartesian', {'twist': [0]*6, 'frame': 'WORLD'}, 1001)
        arm.request('robot/jog/svd', {'dim_speeds': [0, 1]}, 1001)
        arm.request('robot/jog/nullspace', {'joints': [0]*7, 'timeout': 0}, 1001)
        arm.request('robot/movej_ik', {'pose': {}}, 1001)
        arm.request('robot/movel', {'pose': {'position': {}, 'orientation': {}}}, 1001)
    cases = [('jog/joint', {'joints': [0, 0, 0, 0, 0, 0, 1], 'speed': 0.02}),
             ('jog/cartesian', {'twist': [0, 0, 1, 0, 0, 0], 'frame': 'BASE', 'speed': 0.001}),
             ('jog/nullspace', {'joints': [1, 0, 0, 0, 0, 0, 0], 'speed': 0.01}),
             ('jog/svd', {'dim_speeds': [0.01]})]
    for moving, holding in ((left, right), (right, left)):
        initial_pose = moving.request('robot/tcp')['pose']
        for endpoint, body in cases:
            held = positions(holding.state())
            start = positions(moving.state())
            call(moving.prefix + 'robot/' + endpoint, body, holding.headers, 3007)
            for _ in range(8):
                moving.request('robot/' + endpoint, dict(body, timeout=0.15))
                time.sleep(0.035)
            wait_for(lambda: moving.state()['motion_busy'], 2)
            moving.request('robot/active_tool_frame', {'name': ''}, 4005)
            if endpoint == 'jog/joint':
                moving.request('robot/movej', {'joints': start}, -2215)
            # No more feeds: controller must stop without relying on jog/stop.
            wait_for(lambda: not moving.state()['motion_busy'], 4)
            time.sleep(0.1)
            final = positions(moving.state())
            assert error(positions(holding.state()), held) < 0.005
            assert error(final, start) > 0.00001, (moving.side, endpoint, 'no observed motion')
            moving.request('robot/jog/stop', {})
            report['checks'].append({'arm': moving.side, 'jog': endpoint, 'feedback_delta_rad': error(final, start),
                                     'feed_timeout_stopped': True, 'other_arm_held': True})
        jog_pose = moving.request('robot/tcp')['pose']
        for endpoint in ('movej_ik', 'movel', 'movel_fk'):
            before = positions(moving.state())
            held = positions(holding.state())
            target = list(before)
            target[-1] += 0.005
            body = {'joints': target} if endpoint == 'movel_fk' else {'pose': initial_pose if endpoint == 'movej_ik' else jog_pose}
            moving.request('robot/' + endpoint, dict(body, velocity=0.02, acceleration=0.1, jerk=1))
            time.sleep(0.1)
            wait_for(lambda: not moving.state()['motion_busy'], 6)
            delta = error(positions(moving.state()), before)
            assert delta > 0.00001, (moving.side, endpoint, 'no observed motion')
            assert error(positions(holding.state()), held) < 0.005
            if endpoint != 'movel_fk':
                actual = moving.request('robot/tcp')['pose']['position']
                assert max(abs(actual[k] - body['pose']['position'][k]) for k in ('x', 'y', 'z')) < 0.001
            report['checks'].append({'arm': moving.side, 'motion': endpoint, 'feedback_delta_rad': delta,
                                    'completed': True, 'other_arm_held': True})
    jog = {'joints': [0, 0, 0, 0, 0, 0, 1], 'speed': 0.01, 'timeout': 0.5}
    for arm in arms:
        arm.request('robot/jog/joint', jog)
    wait_for(lambda: all(arm.state()['motion_busy'] for arm in arms), 2)
    call(right.prefix + 'robot/jog/stop', {}, left.headers, 3007)
    left.request('robot/jog/stop', {})
    wait_for(lambda: not left.state()['motion_busy'], 2)
    assert right.state()['motion_busy'], 'Stopping left jog stopped right jog'
    right.request('robot/jog/stop', {})
    wait_for(lambda: not right.state()['motion_busy'], 2)
    report['checks'].append('simultaneous independent jogs; explicit jog/stop affects only its arm')
