"""Tests for viewer motion ownership without commanding a robot."""

from collections import deque
import queue
from types import SimpleNamespace
from unittest.mock import Mock

from obot.obot_cmd_motion import MotionTest, MuxOutputTracker, TimingDiagnostics
from obot_msgs.srv import ClickDrive
import pytest


def node():
    n = SimpleNamespace(
        state='moving', active=('F', 1.0, 1.0), pending=deque([('F', 1.0)]),
        command_success=True, failure_reason=None, mux_tracker=MuxOutputTracker(0.5),
        observed_velocity=(0.0, 0.0), progress=0.0, timing=TimingDiagnostics(),
        still_since=None, inputs=queue.Queue(), manual_owner=None,
        manual_deadline=0.0, manual_velocity=(0.0, 0.0), viewer_sequences={},
        viewer_permits={},
        linear_speed=0.2, angular_speed=0.35, max_viewer_linear_speed=0.2,
        max_viewer_angular_speed=0.35, _publish_cmd=Mock(), _fresh_odom=Mock(return_value=True),
        _stopped=Mock(return_value=True), _manual_active=Mock(return_value=False),
        get_logger=Mock(return_value=Mock()),
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=100_000_000_000)))
    n._stop = lambda: MotionTest._stop(n)
    return n


def request(command, sequence, **kwargs):
    r = ClickDrive.Request()
    r.command = command
    r.client_id = 'viewer'
    r.sequence = sequence
    for key, value in kwargs.items():
        setattr(r, key, value)
    return r


def call(n, req):
    if req.command in ('TURN', 'DRIVE', 'MANUAL_BEGIN', 'MANUAL'):
        prepare = request('PREPARE', req.sequence, permit_command=req.command)
        prepare.client_id = req.client_id
        prepared = MotionTest._on_click_drive(n, prepare, ClickDrive.Response())
        if not prepared.accepted:
            return prepared
        req.permit_id = prepared.permit_id
    return MotionTest._on_click_drive(n, req, ClickDrive.Response())


def test_begin_cancels_and_acknowledges_before_motion():
    n = node()
    response = call(n, request('MANUAL_BEGIN', 2))
    assert response.accepted and response.state == 'manual'
    assert n.active is None and not n.pending
    n._publish_cmd.assert_called_once_with(0.0, 0.0)
    assert n.manual_owner == 'viewer' and n.manual_velocity == (0.0, 0.0)


@pytest.mark.parametrize(('linear', 'angular', 'expected'), [
    (1, 0, (0.2, 0.0)), (-1, 0, (-0.2, 0.0)),
    (0, 1, (0.0, 0.35)), (0, -1, (0.0, -0.35)),
])
def test_manual_directions_are_clamped_and_exclusive(linear, angular, expected):
    n = node()
    call(n, request('MANUAL_BEGIN', 1))
    response = call(n, request('MANUAL', 2, linear_direction=linear,
                               angular_direction=angular, linear_speed_mps=2.5,
                               angular_speed_rps=4.0))
    assert response.accepted and n.manual_velocity == expected
    assert not call(n, request('TURN', 3, angle_deg=20.0)).accepted


def test_release_and_delayed_commands_cannot_resume():
    n = node()
    call(n, request('MANUAL_BEGIN', 1))
    assert call(n, request('MANUAL_END', 3)).accepted
    assert not call(n, request('MANUAL', 2, linear_direction=1,
                               linear_speed_mps=0.2, angular_speed_rps=0.35)).accepted
    assert n.manual_owner is None and n.manual_velocity == (0.0, 0.0)
    assert n.active is None and not n.pending


def test_end_before_delayed_begin_blocks_takeover():
    n = node()
    assert call(n, request('MANUAL_END', 2)).accepted
    assert not call(n, request('MANUAL_BEGIN', 1)).accepted
    assert n.manual_owner is None and n.active is None and not n.pending


def test_click_speed_is_frozen_and_clamped_at_acceptance():
    n = node()
    n._stop()
    response = call(n, request('TURN', 1, angle_deg=30.0,
                               linear_speed_mps=2.5, angular_speed_rps=1.5))
    assert response.accepted
    assert n.command_linear_speed == 0.2
    assert n.command_angular_speed == 0.35


def test_manual_lease_expiry_stops_and_does_not_resume(monkeypatch):
    from obot import obot_cmd_motion as motion

    n = node()
    call(n, request('MANUAL_BEGIN', 1))
    call(n, request('MANUAL', 2, linear_direction=1,
                    linear_speed_mps=0.2, angular_speed_rps=0.35))
    n._publish_cmd.reset_mock()
    now = n.manual_deadline + 0.01
    monkeypatch.setattr(motion, 'time', SimpleNamespace(monotonic=lambda: now))
    n.quit_requested = False
    n._read_input = Mock()
    n._handle_inputs = Mock()
    n._start_next = Mock()
    n._publish_state = Mock()
    n._publish_paths = Mock()
    n.last_path_publish = now
    MotionTest._tick(n)
    assert n.manual_owner is None and n.manual_velocity == (0.0, 0.0)
    assert all(call.args == (0.0, 0.0) for call in n._publish_cmd.call_args_list)
    assert n.active is None and not n.pending


def test_manual_heartbeat_requires_fresh_odometry_and_rejects_diagonal():
    n = node()
    call(n, request('MANUAL_BEGIN', 1))
    invalid = call(n, request('MANUAL', 2, linear_direction=1, angular_direction=1,
                              linear_speed_mps=0.2, angular_speed_rps=0.35))
    assert not invalid.accepted
    n._fresh_odom.return_value = False
    stale = call(n, request('MANUAL', 3, linear_direction=1,
                            linear_speed_mps=0.2, angular_speed_rps=0.35))
    assert not stale.accepted and n.manual_velocity == (0.0, 0.0)


def test_expired_takeover_does_not_cancel_or_move(monkeypatch):
    from obot import obot_cmd_motion as motion

    n = node()
    now = [100.0]
    monkeypatch.setattr(motion, 'time', SimpleNamespace(monotonic=lambda: now[0]))
    req = request('MANUAL_BEGIN', 1)
    prep = request('PREPARE', 1, permit_command='MANUAL_BEGIN')
    req.permit_id = MotionTest._on_click_drive(n, prep, ClickDrive.Response()).permit_id
    now[0] += 0.6
    response = MotionTest._on_click_drive(n, req, ClickDrive.Response())
    assert not response.accepted and n.active is not None
    assert response.message == 'Movement permit expired'
    n._publish_cmd.assert_not_called()
