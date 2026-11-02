"""Software-in-the-loop check of the OBot-2 twist_mux lock design.

Runs a REAL twist_mux with twist_mux_obot2.yaml on an isolated localhost-only
ROS domain and drives its inputs. No robot, CAN or motors are involved.
Usage: python3 mux_lock_check.py <path to twist_mux_obot2.yaml>
"""

import os
import subprocess
import sys
import time

os.environ['ROS_DOMAIN_ID'] = '98'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'

import rclpy  # noqa: E402
from geometry_msgs.msg import TwistStamped  # noqa: E402
from std_msgs.msg import Bool  # noqa: E402

YAML = os.path.abspath(sys.argv[1])
results = []


def check(name, ok, detail=''):
    results.append(ok)
    print(f'{"PASS" if ok else "FAIL"}  {name} {detail}')


def start_mux():
    return subprocess.Popen(
        ['ros2', 'run', 'twist_mux', 'twist_mux', '--ros-args',
         '--params-file', YAML, '-r', 'cmd_vel_out:=/mux_out'],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def stop(proc):
    proc.terminate()
    proc.wait(timeout=5)


rclpy.init()
node = rclpy.create_node('mux_lock_check')
pubs = {
    'teleop': node.create_publisher(TwistStamped, '/cmd_vel_teleop', 10),
    'nav': node.create_publisher(TwistStamped, '/cmd_vel_nav', 10),
    'wd': node.create_publisher(TwistStamped, '/cmd_vel_watchdog', 10),
}
lock_pub = node.create_publisher(Bool, '/watchdog_lock', 10)
received = []
node.create_subscription(
    TwistStamped, '/mux_out',
    lambda m: received.append((time.monotonic(), m.twist.linear.x)), 10)


def drive(seconds, teleop=0.5, nav=0.0, wd=None, lock=None):
    """Publish at 20 Hz; returns linear.x values seen on the mux output."""
    start = time.monotonic()
    received.clear()
    while time.monotonic() - start < seconds:
        for key, x in (('teleop', teleop), ('nav', nav), ('wd', wd)):
            if x is None or (key == 'nav' and not nav):
                continue
            msg = TwistStamped()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.twist.linear.x = float(x)
            pubs[key].publish(msg)
        if lock is not None:
            lock_pub.publish(Bool(data=lock))
        rclpy.spin_once(node, timeout_sec=0.05)
    return [x for _, x in received]


def moving(vals):
    return any(abs(v) > 1e-6 for v in vals)


mux = start_mux()
time.sleep(3.0)
if mux.poll() is not None:
    print('ABORT: twist_mux did not start (exit code %s); results would be '
          'meaningless' % mux.returncode)
    sys.exit(2)

# 1. Fresh mux, no lock message ever sent: must be locked (no motion).
vals = drive(2.0, teleop=0.5)
check('1 locked at startup (no watchdog yet)', not moving(vals),
      f'out={sorted(set(vals))}')  # real proof is test 2 + test 7

# 2. Positive control: lock=false lets teleop through. If this fails the
# 'blocked' results below are not trustworthy.
vals = drive(2.0, teleop=0.5, lock=False)
check('2 unlocked when watchdog says false (positive control)', 0.5 in vals,
      f'out={sorted(set(vals))}')
if 0.5 not in vals:
    stop(mux)
    print('ABORT: positive control failed')
    sys.exit(2)

# 3. Watchdog publishes lock=true plus zero stream: teleop blocked.
vals = drive(2.0, teleop=0.5, wd=0.0, lock=True)
check('3 lock=true blocks teleop', not moving(vals),
      f'out={sorted(set(vals))}')
check('3b watchdog zero stream still reaches output',
      len(vals) > 0 and not moving(vals), f'n={len(vals)}')

# 4. Watchdog process dies while lock=false: lock must expire -> blocked.
drive(1.0, teleop=0.5, lock=False)
time.sleep(0.8)  # lock expires (timeout 0.5 s)
vals = drive(2.0, teleop=0.5, lock=None)
check('4 watchdog silence re-locks within timeout', not moving(vals),
      f'out={sorted(set(vals))}')

# 5. Watchdog dies while lock=true: still blocked.
drive(1.0, teleop=0.5, wd=0.0, lock=True)
vals = drive(2.0, teleop=0.5, lock=None)
check('5 silence after lock=true stays blocked', not moving(vals))

# 6. Navigation is masked too.
vals = drive(2.0, teleop=None, nav=0.4, lock=True)
check('6 nav blocked by lock', not moving(vals))

# 7. mux restart while watchdog says unlocked and teleop keeps streaming:
# the new mux must start locked until the first lock message arrives.
drive(1.0, teleop=0.5, lock=False)
stop(mux)
mux = start_mux()
first_motion = None
t0 = time.monotonic()
vals_before_lock = []
end = t0 + 3.0
while time.monotonic() < end:
    # teleop streams; the lock message only starts after 1.5 s (restart gap)
    lock = False if time.monotonic() - t0 > 1.5 else None
    vals = drive(0.1, teleop=0.5, lock=lock)
    if time.monotonic() - t0 <= 1.5:
        vals_before_lock += vals
check('7 restarted mux emits no motion before first lock message',
      not moving(vals_before_lock), f'n={len(vals_before_lock)}')

stop(mux)
node.destroy_node()
rclpy.shutdown()
print('RESULT', 'ALL PASS' if all(results) else 'FAILURES')
sys.exit(0 if all(results) else 1)
