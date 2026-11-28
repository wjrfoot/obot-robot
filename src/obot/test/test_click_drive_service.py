"""Exercise viewer request validation through a real ROS service with mock motion."""

import importlib
import os
from pathlib import Path
import threading
import time
from types import SimpleNamespace

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
    server.create_service(
        ClickDrive, '/click_drive_permit_test',
        lambda req, res: MotionTest._on_click_drive(motion, req, res))
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
    yield motion, send
    executor.shutdown()
    thread.join(timeout=5.0)
    client_node.destroy_node()
    server.destroy_node()
    rclpy.shutdown()


def prepare(send, command, sequence, client_id='viewer'):
    req = request('PREPARE', sequence, permit_command=command)
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
