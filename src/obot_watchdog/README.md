# OBot-2 ODrive S1 watchdog

The watchdog detects S1 faults, records controller diagnostics, attempts
bounded fault recovery where configured, verifies closed-loop control, and
logs/notifies the operator. It operates independently of joystick and motion
control: it does not subscribe to `/joy`, publish velocity commands, or
control `twist_mux`. The joystick and navigation routes remain owned by the
existing teleop, navigation, and `twist_mux` components.

The OBot-2 controllers use CAN node IDs 61 (left) and 62 (right) on `can0` at
1 Mbps. OBot-1's separate watchdog and node IDs are not changed.

## Detection and recovery

The watchdog polls each controller every second for `Get_Error` and bus
voltage, while receiving heartbeats and checking the CAN link. It records
controller ID, heartbeat/error details, disarm reason, axis state, bus
voltage when fresh, CAN state, and timestamp. Notifications are published on
`/obot/watchdog_event`; the existing laptop notifier continues to display
them.

Automatic recovery is enabled (`dry_run: false`) and bounded to three
attempts per fault episode, with a five-second retry interval and a
three-second verification timeout. Only `DC_BUS_OVER_VOLTAGE` (`0x100`) is
eligible for error clearing. Other active S1 errors are reported and are not
automatically cleared. For an eligible fault the watchdog sends
`Clear_Errors`, waits the configured command gap, requests
`CLOSED_LOOP_CONTROL` (state 8), and requires fresh heartbeat and `Get_Error`
data showing state 8 and no active/disarm errors. Healthy controllers are
not sent state-changing commands.

The diagnostic polling loop transmits the left `Get_Error` immediately and
stages the right request for 50 ms later using a non-blocking ROS timer. A
10 ms timer checks the deadline. If a second right-side diagnostic arrives
while one is still queued, the outstanding request is sent immediately and
the overlap is logged before the new request is scheduled. Shutdown cancels
the timer and clears its pending request.

CAN link errors, missing/stale heartbeats, and missing `Get_Error` replies are
reported as communication faults. Mutating CAN commands are not issued while
the observed CAN link is not `ERROR-ACTIVE`.

## Configuration and logs

Configuration: `config/obot2_watchdog.yaml`.

Important values:

| Parameter | Value |
|---|---:|
| `monitor_interval` | `1.0` s |
| `get_error_miss_limit` | `5` |
| `recoverable_error_mask` | `256` (`DC_BUS_OVER_VOLTAGE`) |
| `max_recoveries` | `3` |
| `retry_interval_s` | `5.0` s |
| `dry_run` | `false` |

Fault records are appended to
`/home/pi/obot_logs/obot2_watchdog_faults.jsonl`. ROS notifications retain
the `/obot/watchdog_event` topic and tagged `[INFO]`, `[WARNING]`, or
`[CRITICAL]` messages consumed by `obot-watchdog-notify` on the operator
laptop. The state summary is published on `/obot2/watchdog/state`.

## Build and tests

From the repository root in a sourced ROS 2 Jazzy environment:

```sh
colcon build --packages-select obot_watchdog --symlink-install
python3 -m pytest src/obot_watchdog/test -q
```

Tests use mocked ROS timers and CAN transmission; passing tests do not
validate hardware safety or the installed S1 firmware configuration.

## Deployment

No deployment is performed by this repository change. After review and
approval, rebuild on the robot and restart `obot2-watchdog.service` to load
the new watchdog. If the OBot-2 bringup uses `twist_mux_obot2.yaml`, restart
`obot.service` to load the removal of its obsolete watchdog velocity and lock
entries. Do not change the normal joystick/navigation routes. Verify actual
CAN bitrate, node IDs, ODrive error bits, and recovery behavior on a controlled
bench before enabling operation.

The watchdog is diagnostic/recovery support, not an emergency stop. The
physical E-stop remains the emergency-stop mechanism.
