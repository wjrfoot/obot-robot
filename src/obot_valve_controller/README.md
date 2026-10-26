# OBot spray valve controller

This package provides a robot-side ROS 2 action server and a separate
development-laptop action client for the 12 V spray solenoid. It does not
change the drivetrain, motor-controller, CAN, or existing teleoperation
nodes.

## Interfaces and behavior

The action server exposes `/obot/spray` using
`obot_valve_interfaces/action/SprayValve`:

| `spray_mode` | Button | Default duration |
| --- | --- | ---: |
| 0 | Circle | 1.0 s |
| 1 | Square | 3.0 s |
| 2 | Triangle | 5.0 s |

The default parameters are in `config/valve_controller.yaml`. Durations may be
zero but must be finite and no greater than 300 seconds. The server reports
the last commanded valve state periodically on `/obot/valve/state`
(`std_msgs/msg/Bool`) and publishes one
`obot_valve_interfaces/msg/ValveCompletion` event for each accepted action on
`/obot/valve/completion`.

The server reserves an action as soon as it accepts the goal, rejects invalid
modes and overlapping requests, and uses a monotonic clock for timing. It
commands the GPIO inactive at startup, completion, cancellation, execution
errors, and normal node shutdown (GPIO 22 low with the default active-HIGH
configuration). The line is requested once and released when the node shuts
down. The default GPIO configuration is `/dev/gpiochip4`, line 22, active
HIGH, controlled through Ubuntu 24.04's libgpiod 1.x Python API.

## Build and test

From the workspace, source the ROS 2 Jazzy environment and build the new
packages:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/obot_ws
export ROS_DOMAIN_ID=5
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-up-to obot_valve_controller
source install/setup.bash
```

Run the hardware-free tests without opening a GPIO device:

```bash
python3 -m pytest src/obot_valve_controller/test
```

## Safe mock-mode server

To run the action server without activating or requesting the physical relay
line:

```bash
ros2 launch obot_valve_controller valve_server.launch.py mock_gpio:=true
```

The mock backend records logical GPIO commands only. It is also used by the
unit tests. Do not omit `mock_gpio:=true` when the intent is to run the server
without hardware; the normal launch defaults to the configured real GPIO line.

## Robot-side hardware server

After verifying the board wiring, GPIO permissions, relay polarity, and
12 V valve power path, run on the robot:

```bash
ros2 launch obot_valve_controller valve_server.launch.py
```

This opens the actual GPIO output. Check `/obot/valve/state` and
`/obot/valve/completion` during supervised bench testing before spraying.
Parameter overrides can be supplied through the launch system or YAML. Keep
the GPIO output and valve fail-safe wiring appropriate for the application.

## Laptop-side PS4 client

Use the existing joystick launch/driver that already publishes `/joy`; this
client does **not** start another `joy_node`. Before enabling spraying, inspect
the live zero-based button positions:

```bash
ros2 topic echo /joy
```

Press each face button separately and note the corresponding position in the
`buttons` array. Then launch the client on the laptop with those confirmed
indices (replace the tokens below with the observed numeric indices):

```bash
ros2 launch obot_valve_controller ps4_valve_client.launch.py \
  circle_button:=CIRCLE_INDEX square_button:=SQUARE_INDEX \
  triangle_button:=TRIANGLE_INDEX cross_button:=CROSS_INDEX
```

The defaults are `-1` (unconfigured), so no face-button index is assumed. The
client detects rising edges, ignores new spray presses while a goal is active
or awaiting acceptance, and allows Cross to request cancellation throughout
that interval. It does not permit another spray until the prior action returns
a terminal result.

Rumble follows the robot's commanded state, not button presses or action
feedback. While the reported valve state is open, the client publishes
`sensor_msgs/msg/JoyFeedbackArray` rumble feedback on `/joy/set_feedback`,
refreshing it at the configured interval. It sends zero on a closed state,
stale state feedback (default timeout 0.6 s), or client shutdown. Default
intensity is 0.5. Confirm that the installed joystick driver subscribes to
`/joy/set_feedback` and supports sustained rumble before use; this workspace's
current ROS installation does not include the `joy` package, so driver-level
force-feedback behavior must be verified on the deployment laptop.

## Safety limitations

The action server's timer and GPIO output are robot-side, so a laptop or ROS
client disconnect does not extend a spray: the server still closes the valve
when the requested duration expires. Software shutdown handling requests the
output OFF during normal process shutdown, but it cannot guarantee valve
closure after a hardware fault, process crash, or power failure. Independent
hardware safety protections are required. This software alone is not suitable
for unattended pesticide spraying.
