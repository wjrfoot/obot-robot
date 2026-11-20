import math
import time

import pytest
import rclpy
from nav_msgs.msg import Odometry, Path
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger

from obot.path_recorder import PathHistory, PathRecorder


def odom(x=0.0, y=0.0, yaw=0.0, frame='odom', stamp=10):
    msg = Odometry()
    msg.header.frame_id = frame
    msg.header.stamp.sec = stamp
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.orientation.z = math.sin(yaw / 2)
    msg.pose.pose.orientation.w = math.cos(yaw / 2)
    return msg


def test_sampling_and_pose_copy():
    history = PathHistory(0.05, 0.15, 5)
    first = odom()
    assert history.add(first)
    assert not history.add(odom(x=0.04))
    assert history.add(odom(x=0.06))
    assert history.add(odom(x=0.06, yaw=0.2))
    assert len(history.points) == 3
    assert history.points[0].header.frame_id == 'odom'
    assert history.points[0].header.stamp.sec == 10
    first.pose.pose.position.x = 100
    assert history.points[0].pose.position.x == 0


def test_bounded_history_clear_and_discontinuities():
    history = PathHistory(0.05, 0.15, 2)
    for x in (0.0, 1.0, 2.0):
        assert history.add(odom(x=x))
    assert [p.pose.position.x for p in history.points] == [1, 2]
    assert history.add(odom(x=3, frame='new'))
    assert [p.header.frame_id for p in history.points] == ['new']
    assert history.add(odom(x=4, frame='new', stamp=5))
    assert len(history.points) == 1
    history.clear()
    assert history.add(odom(x=4, frame='new', stamp=6))
    assert len(history.points) == 1


def test_invalid_odom_does_not_change_history():
    history = PathHistory(0.05, 0.15, 5)
    assert history.add(odom())
    assert not history.add(odom(frame=''))
    assert not history.add(odom(x=float('nan')))
    invalid = odom()
    invalid.pose.pose.orientation.w = 0.0
    assert not history.add(invalid)
    assert len(history.points) == 1


@pytest.mark.parametrize('args', [
    (-1, 0.1, 2), (0.1, -1, 2), (0.1, 0.1, 0),
    (float('nan'), 0.1, 2),
])
def test_invalid_parameters_rejected(args):
    with pytest.raises(ValueError):
        PathHistory(*args)


def test_late_subscriber_receives_path_and_clear():
    rclpy.init()
    recorder = PathRecorder()
    observer = rclpy.create_node('path_test_observer')
    executor = SingleThreadedExecutor()
    executor.add_node(recorder)
    executor.add_node(observer)
    received = []
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    try:
        odom_pub = observer.create_publisher(Odometry, '/diff_drive_controller/odom', 10)
        deadline = time.monotonic() + 5
        while odom_pub.get_subscription_count() == 0 and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.1)
        assert odom_pub.get_subscription_count() > 0
        odom_pub.publish(odom(x=1.0))
        deadline = time.monotonic() + 5
        while not recorder.history.points and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.1)
        assert len(recorder.history.points) == 1
        recorder._publish()
        sub = observer.create_subscription(Path, '/obot/path', received.append, qos)
        deadline = time.monotonic() + 5
        while not received and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.1)
        assert received and len(received[-1].poses) == 1
        assert received[-1].header.frame_id == 'odom'
        client = observer.create_client(Trigger, '/obot/clear_path')
        assert client.wait_for_service(timeout_sec=5)
        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + 5
        while not future.done() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.1)
        assert future.done() and future.result().success
        deadline = time.monotonic() + 5
        while (not received or received[-1].poses) and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.1)
        assert received and not received[-1].poses
        recorder._on_odom(odom(x=1.0))
        assert len(recorder.history.points) == 1
        observer.destroy_subscription(sub)
    finally:
        executor.shutdown()
        recorder.destroy_node()
        observer.destroy_node()
        rclpy.shutdown()
