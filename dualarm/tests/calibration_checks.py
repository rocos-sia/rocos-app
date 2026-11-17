"""Real feedback/HTTP checks; synthetic geometry belongs to the offline solver test."""
from concurrent.futures import ThreadPoolExecutor


def check_calibration(arms, call, wait_for, report):
    assert call('/api/arms')['capabilities']['six_point_calibration']
    for arm in arms:
        wait_for(lambda: max(abs(j['velocity']) for j in arm.state()['joint_states']) < 0.0008)
    body = {'name': 'calibration_probe', 'index': 1}
    call(arms[0].prefix + 'calibration/pose', body, {}, 3006)
    call(arms[1].prefix + 'calibration/pose', body, arms[0].headers, 3007)
    call('/api/arms/wrong/calibration/pose', body, arms[0].headers, 1001)
    arms[0].request('calibration/pose', {**body, 'pose': {}}, 1001)
    arms[0].request('calibration/pose', {**body, 'index': 2}, 4001)
    with ThreadPoolExecutor(max_workers=2) as pool:
        sessions = list(pool.map(lambda arm: arm.request('calibration/pose', body), arms))
    assert sessions[0]['session_id'] != sessions[1]['session_id']
    for i, (arm, session) in enumerate(zip(arms, sessions)):
        other = arms[1 - i]
        sid = session['session_id']
        assert session['captured'] == 1 and session['result'] is None
        assert len(session['samples']) == 6 and session['samples'][1:] == [None] * 5
        sample = session['samples'][0]
        assert len(sample['joints']) == 7 and sample['timestamp'] > 0
        feedback = [j['position'] for j in arm.state()['joint_states']]
        assert max(abs(a - b) for a, b in zip(sample['joints'], feedback)) < 0.002
        assert call(arm.prefix + 'calibration/result?session_id=' + sid) == session
        other.request('calibration/result?session_id=' + sid, expected=4004)
        other.request('calibration/run', {'session_id': sid}, 4004)
        other.request('calibration/save', {'session_id': sid}, 4004)
        arm.request('calibration/run', {'session_id': sid}, 4001)
        arm.request('calibration/save', {'session_id': sid}, 4001)
        arm.request('calibration/run', {'session_id': sid, 'max_error': 0}, 1001)
        for index in range(2, 7):
            arm.request('calibration/pose', {**body, 'index': index, 'session_id': sid})
        # Repeated stationary poses cannot determine a six-point tool.
        arm.request('calibration/run', {'session_id': sid}, 4002)
        assert arm.request('calibration/result?session_id=' + sid)['result'] is None
        arm.request('calibration/save', {'session_id': sid}, 4001)
        assert body['name'] not in arm.request('robot/tool_frames')['names']
        arm.request('control/release', {})
        arm.acquire()
        arm.request('calibration/run', {'session_id': sid}, 4004)
        arm.request('calibration/save', {'session_id': sid}, 4004)
        arm.request('calibration/pose', {**body, 'session_id': sid}, 4004)
        assert arm.request('calibration/result?session_id=' + sid)['captured'] == 6
        arm.request('robot/frames/save', {})
        arm.request('robot/frames/load', {})
        arm.request('calibration/result?session_id=' + sid, expected=4004)
        report['checks'].append({'calibration': arm.side,
            'real_feedback_capture': True, 'cross_arm_and_owner_isolation': True,
            'degenerate_geometry_rejected': True, 'frames_reload_invalidates': True})
