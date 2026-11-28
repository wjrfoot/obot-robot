"""Exercise viewer request validation through a real ROS service with mock motion."""

from concurrent.futures import Future
import importlib
import os
from pathlib import Path
import threading
import time
from types import SimpleNamespace
from unittest.mock import Mock

from obot.obot_cmd_motion import MotionTest
from obot_msgs.srv import ClickDrive
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from test_click_drive_manual import node, request


@pytest.fixture
def service():
    rclpy.init()
    server = rclpy.create_node('mock_click_drive_server')
    client_node = rclpy.create_node('mock_click_drive_client')
    motion = node()
    motion.state = 'idle'
    motion.active = None
    motion.pending.clear()
    received = []

    def handle(req, res):
        if req.command == 'MANUAL':
            received.append((req.linear_direction, req.angular_direction))
        return MotionTest._on_click_drive(motion, req, res)

    server.create_service(
        ClickDrive, '/click_drive_permit_test', handle)
    client = client_node.create_client(ClickDrive, '/click_drive_permit_test')
    executor = SingleThreadedExecutor()
    executor.add_node(server)
    thread = threading.Thread(target=executor.spin)
    thread.start()
    assert client.wait_for_service(timeout_sec=5.0)

    def send(req):
        future = client.call_async(req)
        rclpy.spin_until_future_complete(client_node, future, timeout_sec=5.0)
        assert future.done(), 'mock service did not respond'
        return future.result()

    send.client = client
    send.node = client_node
    send.received = received
    yield motion, send
    executor.shutdown()
    thread.join(timeout=5.0)
    client_node.destroy_node()
    server.destroy_node()
    rclpy.shutdown()


def prepare(send, command, sequence, client_id='viewer', continuation_of=''):
    req = request('PREPARE', sequence, permit_command=command,
                  permit_id=continuation_of)
    req.client_id = client_id
    return send(req)


def move(send, command, sequence, permit, client_id='viewer', **kwargs):
    req = request(command, sequence, permit_id=permit, **kwargs)
    req.client_id = client_id
    return send(req)


@pytest.mark.parametrize('offset', [-3600.0, 3600.0])
@pytest.mark.parametrize('stop_command', ['STOP', 'MANUAL_END'])
def test_click_and_directional_requests_ignore_independent_ros_clocks(
        service, offset, stop_command):
    motion, send = service
    motion.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=int(offset * 1e9)))
    turn = prepare(send, 'TURN', 1)
    assert turn.accepted
    result = move(send, 'TURN', 1, turn.permit_id, angle_deg=20.0)
    assert result.accepted and list(motion.pending) == [('r', 20.0)]
    assert send(request('STOP', 2)).accepted
    begin = prepare(send, 'MANUAL_BEGIN', 3)
    assert move(send, 'MANUAL_BEGIN', 3, begin.permit_id).accepted
    manual = prepare(send, 'MANUAL', 4)
    assert move(send, 'MANUAL', 4, manual.permit_id, linear_direction=1,
                linear_speed_mps=0.2, angular_speed_rps=0.35).accepted
    assert motion.manual_velocity == (0.2, 0.0)
    delayed = prepare(send, 'MANUAL', 5)
    assert send(request(stop_command, 1)).accepted
    result = move(send, 'MANUAL', 5, delayed.permit_id, linear_direction=1,
                  linear_speed_mps=0.2, angular_speed_rps=0.35)
    assert not result.accepted and result.message == 'Movement permit was canceled'
    assert motion.manual_owner is None and motion.manual_velocity == (0.0, 0.0)


@pytest.mark.parametrize(('button', 'expected', 'velocity'), [
    ('FORWARD', (1, 0), (0.2, 0.0)), ('REVERSE', (-1, 0), (-0.2, 0.0)),
    ('LEFT', (0, 1), (0.0, 0.35)), ('RIGHT', (0, -1), (0.0, -0.35)),
])
def test_actual_viewer_direction_over_ros_service(
        service, monkeypatch, button, expected, velocity):
    import cv2
    import obot

    viewer_source = Path(os.environ.get(
        'OBOT_VIEWER_SOURCE',
        str(Path(__file__).resolve().parents[4] / 'obot_ws/src/obot/obot')))
    if not (viewer_source / 'dual_compressed_viewer.py').is_file():
        pytest.skip('Laptop viewer source is not available')
    monkeypatch.setattr(obot, '__path__', [*obot.__path__, str(viewer_source)])
    module = importlib.import_module('obot.dual_compressed_viewer')
    motion, send = service
    viewer = SimpleNamespace(
        _viewer_id='viewer', _viewer_sequence=0, _motion_session_ready=True,
        directional_speed=0.2,
        directional_settings={name.removeprefix('directional.'): value
                              for name, value in module.DIRECTIONAL_DEFAULTS.items()},
        _manual_client=send.client, _manual_button=None, _manual_owned=False,
        _manual_future=None, _manual_kind=None, _manual_future_started=0.0,
        _directional_hits={button: (10, 10, 50, 40)}, speed_presets={},
        directional_enable_motion=True, _click_future=None, _click_request_kind=None,
        tracking_worker=Mock())
    for name in ('_viewer_request', '_manual_send', '_manual_stop', '_manual_press',
                 '_clear_selected_target', '_handle_motion_mouse'):
        setattr(viewer, name, getattr(module.DualCompressedViewer, name).__get__(viewer))

    def await_response():
        operation = viewer._manual_future
        deadline = time.monotonic() + 3.0
        while not operation.done() and time.monotonic() < deadline:
            rclpy.spin_once(send.node, timeout_sec=0.005)
        assert operation.done()
        return operation.result()

    assert viewer._handle_motion_mouse(cv2.EVENT_LBUTTONDOWN, 20, 20, cv2.EVENT_FLAG_LBUTTON)
    assert await_response().accepted
    assert viewer._manual_send('MANUAL')
    result = await_response()
    assert send.received[-1] == expected
    assert result.accepted, result.message
    assert motion.manual_velocity == pytest.approx(velocity)


def test_duplicates_reordering_expiry_and_stop_barriers(service, monkeypatch):
    from obot import obot_cmd_motion as motion_module

    motion, send = service
    now = [100.0]
    monkeypatch.setattr(motion_module, 'time', SimpleNamespace(monotonic=lambda: now[0]))
    first = prepare(send, 'TURN', 1)
    assert first.accepted
    duplicate = prepare(send, 'TURN', 1)
    assert not duplicate.accepted and 'sequence' in duplicate.message
    second = prepare(send, 'DRIVE', 2)
    stale = move(send, 'TURN', 1, first.permit_id, angle_deg=20.0)
    assert not stale.accepted and 'Out-of-order' in stale.message
    now[0] += 0.6
    expired = move(send, 'DRIVE', 2, second.permit_id, distance_m=1.0)
    assert not expired.accepted and expired.message == 'Movement permit expired'
    permit = prepare(send, 'TURN', 3)
    assert send(request('STOP', 4)).accepted
    canceled = move(send, 'TURN', 3, permit.permit_id, angle_deg=20.0)
    assert not canceled.accepted and canceled.message == 'Movement permit was canceled'
    delayed_prepare = prepare(send, 'MANUAL_BEGIN', 3)
    assert not delayed_prepare.accepted
    assert not motion.pending and motion.manual_velocity == (0.0, 0.0)
    assert send(request('STOP', 1)).accepted
    assert send(request('MANUAL_END', 1)).accepted
    anonymous = request('TURN', 1, angle_deg=20.0)
    anonymous.client_id = ''
    denied = send(anonymous)
    assert not denied.accepted and denied.message == 'Movement request requires a viewer identity'


def test_single_use_and_restart_with_controller_running(service):
    motion, send = service
    first = prepare(send, 'TURN', 1, 'old-viewer')
    assert move(send, 'TURN', 1, first.permit_id, 'old-viewer', angle_deg=20.0).accepted
    replay = move(send, 'TURN', 1, first.permit_id, 'old-viewer', angle_deg=20.0)
    assert not replay.accepted and replay.message == 'Movement permit was used'
    pending = prepare(send, 'DRIVE', 2, 'old-viewer')
    startup_stop = request('STOP', 0)
    startup_stop.client_id = ''
    assert send(startup_stop).accepted
    delayed = move(send, 'DRIVE', 2, pending.permit_id, 'old-viewer', distance_m=1.0)
    assert not delayed.accepted and delayed.message == 'Movement permit was canceled'
    restarted = prepare(send, 'TURN', 1, 'new-viewer')
    assert move(send, 'TURN', 1, restarted.permit_id, 'new-viewer', angle_deg=5.0).accepted
    assert list(motion.pending) == [('r', 5.0)]


def test_status_correlates_completion_and_stop_with_accepted_request(service):
    motion, send = service
    permit = prepare(send, 'TURN', 1)
    assert move(send, 'TURN', 1, permit.permit_id, angle_deg=20.0).accepted
    queued = send(request('STATUS', 2))
    assert queued.state == 'queued'
    assert queued.message.startswith('result=success, request=viewer:1,')
    motion.pending.clear()
    complete = send(request('STATUS', 3))
    assert complete.state == 'idle'
    assert complete.message.startswith('result=success, request=viewer:1,')
    assert send(request('STOP', 4)).accepted
    assert 'request=none,' in send(request('STATUS', 5)).message


def test_large_finite_drive_is_accepted(service):
    motion, send = service
    permit = prepare(send, 'DRIVE', 1)
    response = move(send, 'DRIVE', 1, permit.permit_id, distance_m=12.0)
    assert response.accepted
    assert list(motion.pending) == [('F', 12.0)]


def test_stopped_queued_turn_cannot_report_success(service):
    _, send = service
    permit = prepare(send, 'TURN', 1)
    assert move(send, 'TURN', 1, permit.permit_id, angle_deg=20.0).accepted
    assert send(request('STOP', 2)).accepted
    status = send(request('STATUS', 3))
    assert status.state == 'idle'
    assert status.message.startswith('result=failed, request=none,')


def test_stop_clears_active_pending_and_waiting_cli_motion(service):
    motion, send = service
    motion.active = ('F', 3.0, 3.0)
    motion.state = 'moving'
    motion.pending.append(('r', 20.0))
    motion.inputs.put(('F', 5.0))
    assert send(request('STOP', 1)).accepted
    assert motion.active is None and not motion.pending and motion.inputs.empty()
    MotionTest._handle_inputs(motion)
    assert not motion.pending


@pytest.mark.parametrize('stop_command', ['STOP', 'MANUAL_END'])
def test_stopping_after_continuation_permit_blocks_delayed_drive(service, stop_command):
    motion, send = service
    turn = prepare(send, 'TURN', 1)
    assert move(send, 'TURN', 1, turn.permit_id, angle_deg=20.0).accepted
    motion.pending.clear()
    permit = prepare(send, 'DRIVE', 2, continuation_of='viewer:1')
    assert permit.accepted
    stop = request(stop_command, 1)
    stop.client_id = 'another-viewer'
    assert send(stop).accepted
    result = move(send, 'DRIVE', 2, permit.permit_id, distance_m=2.0,
                  permit_command='viewer:1')
    assert not result.accepted and result.message == 'Movement permit was canceled'
    assert not motion.pending
    explicit = prepare(send, 'DRIVE', 3)
    assert move(send, 'DRIVE', 3, explicit.permit_id, distance_m=2.0).accepted
    assert list(motion.pending) == [('F', 2.0)]


def test_continuation_cannot_survive_manual_takeover(service):
    motion, send = service
    turn = prepare(send, 'TURN', 1)
    assert move(send, 'TURN', 1, turn.permit_id, angle_deg=20.0).accepted
    motion.pending.clear()
    permit = prepare(send, 'DRIVE', 2, continuation_of='viewer:1')
    assert permit.accepted
    manual = prepare(send, 'MANUAL_BEGIN', 1, client_id='other')
    assert move(send, 'MANUAL_BEGIN', 1, manual.permit_id, client_id='other').accepted
    result = move(send, 'DRIVE', 2, permit.permit_id, distance_m=2.0,
                  permit_command='viewer:1')
    assert not result.accepted and 'continuation canceled' in result.message
    assert not motion.pending


def test_continuation_cannot_strip_dependency_from_permit(service):
    motion, send = service
    turn = prepare(send, 'TURN', 1)
    assert move(send, 'TURN', 1, turn.permit_id, angle_deg=20.0).accepted
    motion.pending.clear()
    permit = prepare(send, 'DRIVE', 2, continuation_of='viewer:1')
    assert permit.accepted
    result = move(send, 'DRIVE', 2, permit.permit_id, distance_m=2.0)
    assert not result.accepted and 'does not match' in result.message
    assert not motion.pending


@pytest.mark.parametrize('stop_mode', ['none', 'external', 'universal'])
def test_viewer_combined_go_uses_confirmed_turn_and_fresh_distance(
        service, monkeypatch, stop_mode):
    import obot

    viewer_source = Path(os.environ.get(
        'OBOT_VIEWER_SOURCE',
        str(Path(__file__).resolve().parents[4] / 'obot_ws/src/obot/obot')))
    if not (viewer_source / 'dual_compressed_viewer.py').is_file():
        pytest.skip('Laptop viewer source is not available')
    monkeypatch.setattr(obot, '__path__', [*obot.__path__, str(viewer_source)])
    module = importlib.import_module('obot.dual_compressed_viewer')
    motion, send = service
    settings = {key.removeprefix('directional.'): value
                for key, value in module.DIRECTIONAL_DEFAULTS.items()}
    v = SimpleNamespace(
        _manual_button=None, click_state='selected', click_enable_motion=True,
        selected_target=module.TargetTrackerState(status='tracking', bearing_deg=20.0),
        _pending_go=None, _click_future=None, _click_request_kind=None,
        _click_client=send.client, _motion_session_ready=True,
        _viewer_id='viewer', _viewer_sequence=0, _motion_result_id=None,
        directional_speed=0.2, directional_settings=settings,
        _click_saw_turn_motion=False, _click_saw_drive_motion=False,
        _click_last_status=0.0, _go_requested_distance_m=None, _go_start_pose=None,
        _latest_odom_pose=(0.0, 0.0, 0.0), operator_units='m',
        tracking_worker=Mock(), click_message='',
        _manual_stop=Mock(), spray=Mock(),
        _go_distance_from_lidar=Mock(return_value=8.0))
    for name in ('_viewer_request', '_click_request', '_click_poll', '_begin_go',
                 '_start_go_drive', '_request_drive', '_clear_selected_target',
                 '_universal_stop'):
        setattr(v, name, getattr(module.DualCompressedViewer, name).__get__(v))

    def wait_for(condition):
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            rclpy.spin_once(send.node, timeout_sec=0.01)
            v._click_poll()
            if condition():
                return
        pytest.fail(f'viewer did not advance: {v.click_state} {v.click_message}')

    v._begin_go(None)
    wait_for(lambda: v._click_saw_turn_motion)
    assert list(motion.pending) == [('r', 20.0)]
    v._go_distance_from_lidar.assert_not_called()
    motion.pending.clear()
    if stop_mode != 'none':
        # A completed STATUS response can be delayed across another client's STOP.
        completed = send(request('STATUS', 2))
        response = Future()
        v._click_future = response
        v._click_request_kind = 'STATUS'
        if stop_mode == 'external':
            stop = request('STOP', 1)
            stop.client_id = 'other-viewer'
            assert send(stop).accepted
            response.set_result(completed)
            v._click_poll()
            wait_for(lambda: v.click_state == 'ready' and v._click_future is None)
            assert 'rejected' in v.click_message
        else:
            v._universal_stop()
            v.spray.cancel_all.assert_called_once()
            wait_for(lambda: v.click_state == 'idle' and v._click_future is None)
            response.set_result(completed)
            v._click_poll()
        assert not motion.pending
        v._begin_go(2.0)
        wait_for(lambda: bool(motion.pending))
        assert list(motion.pending) == [('F', 2.0)]
    else:
        wait_for(lambda: bool(motion.pending))
        assert list(motion.pending) == [('F', 8.0)]
        assert v._go_requested_distance_m == 8.0
    if stop_mode == 'universal':
        v._go_distance_from_lidar.assert_not_called()
    else:
        v._go_distance_from_lidar.assert_called_once()


@pytest.mark.parametrize('offset', [-3600.0, 3600.0])
def test_actual_viewer_requests_against_mock_motion_service(service, monkeypatch, offset):
    import obot

    viewer_source = Path(os.environ.get(
        'OBOT_VIEWER_SOURCE',
        str(Path(__file__).resolve().parents[4] / 'obot_ws/src/obot/obot')))
    if not (viewer_source / 'dual_compressed_viewer.py').is_file():
        pytest.skip('Laptop viewer source is not available')
    monkeypatch.setattr(obot, '__path__', [*obot.__path__, str(viewer_source)])
    viewer_module = importlib.import_module('obot.dual_compressed_viewer')
    motion, send = service
    motion.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=int(-offset * 1e9)))
    viewer = SimpleNamespace(
        _viewer_id='actual-viewer', _viewer_sequence=0, _motion_session_ready=False,
        directional_speed=0.2,
        directional_settings={
            name.removeprefix('directional.'): value
            for name, value in viewer_module.DIRECTIONAL_DEFAULTS.items()},
        _click_client=send.client, _click_future=None, _click_request_kind=None,
        _manual_client=send.client, _manual_button=None, _manual_owned=False,
        _manual_future=None, _manual_kind=None, _manual_future_started=0.0,
        _manual_last_send=0.0, combined_window='test', show_side_by_side=True,
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=int(offset * 1e9))))
    for name in ('_viewer_request', '_click_request', '_manual_send',
                 '_manual_stop', '_manual_tick'):
        setattr(viewer, name, getattr(viewer_module.DualCompressedViewer, name).__get__(viewer))
    monkeypatch.setattr(viewer_module, 'window_has_focus', lambda _: True)
    assert viewer._click_request('TURN', 15.0)
    deadline = time.monotonic() + 3.0
    while not viewer._click_future.done() and time.monotonic() < deadline:
        rclpy.spin_once(send.node, timeout_sec=0.005)
    assert viewer._click_future.done() and viewer._click_future.result().accepted
    assert list(motion.pending) == [('r', 15.0)]
    assert send(viewer._viewer_request('STOP')).accepted
    viewer._click_future = None
    assert viewer._click_request('DRIVE', distance=1.0)
    deadline = time.monotonic() + 3.0
    while not viewer._click_future.done() and time.monotonic() < deadline:
        rclpy.spin_once(send.node, timeout_sec=0.005)
    assert viewer._click_future.done() and viewer._click_future.result().accepted
    assert list(motion.pending) == [('F', 1.0)]
    assert send(viewer._viewer_request('STOP')).accepted
    viewer._manual_button = 'FORWARD'
    assert viewer._manual_send('MANUAL_BEGIN')
    deadline = time.monotonic() + 3.0
    while motion.manual_velocity == (0.0, 0.0) and time.monotonic() < deadline:
        rclpy.spin_once(send.node, timeout_sec=0.005)
        viewer._manual_tick()
    assert motion.manual_velocity == (0.2, 0.0)
    end = time.monotonic() + 1.0
    while time.monotonic() < end:
        time.sleep(1.0 / 30.0)
        rclpy.spin_once(send.node, timeout_sec=0.005)
        viewer._manual_tick()
        assert time.monotonic() < motion.manual_deadline
    viewer._manual_stop()
    deadline = time.monotonic() + 3.0
    while motion.manual_owner is not None and time.monotonic() < deadline:
        rclpy.spin_once(send.node, timeout_sec=0.005)
    assert motion.manual_owner is None and motion.manual_velocity == (0.0, 0.0)
