# OBot-2 watchdog verification procedure

Status: **TEST PLAN ONLY — NOT EXECUTED ON HARDWARE.**

Unit tests use mocked ROS timers and CAN I/O and do not establish hardware safety. Do not deploy or exercise fault recovery on a robot until the code and this procedure have been reviewed and the E-stop and bench setup are confirmed.

## 1. Software checks

From the repository root in a sourced ROS 2 Jazzy environment:

```sh
colcon build --packages-select obot_watchdog --symlink-install
python3 -m pytest src/obot_watchdog/test -q
```

Confirm the tests cover:

- Per-controller fault detection and diagnostic records.
- `Clear_Errors` and state-8 requests.
- Fresh telemetry verification.
- Bounded retries.
- CAN communication faults.
- Event notifications.
- The 50 ms left/right `Get_Error` stagger.
- Overlapping diagnostic requests.
- Health classification and voltage publication.
- Stale telemetry and unavailable CAN communication.

Inspect the configured command values and limits in `src/obot_watchdog/config/obot2_watchdog.yaml`.

Recovery is enabled in the current configuration, but only the configured recoverable error mask (`0x100`, `DC_BUS_OVER_VOLTAGE`) is automatically cleared. Confirm that the installed ODrive S1 firmware uses the expected error-bit definitions before deployment.

## 2. Communication and routing configuration

Confirm OBot-2 uses:

- CAN node IDs 61 (left) and 62 (right).
- `can0` at 1 Mbps.
- The expected CAN termination and wiring.

Confirm that `twist_mux_obot2.yaml` retains both `/cmd_vel_teleop` and `/cmd_vel_nav` and contains no watchdog velocity topic or lock.

The watchdog must not subscribe to `/joy` or publish velocity commands.

The node logs fault diagnostics to `/home/pi/obot_logs/obot2_watchdog_faults.jsonl` and publishes tagged notifications on `/obot/watchdog_event`. The existing operator-laptop `obot-watchdog-notify` process remains responsible for popup display.

### Diagnostic topics

Verify that the following topics are available:

| Topic | Expected type | Purpose |
|---|---|---|
| `/obot2/watchdog/state` | `std_msgs/String` | Internal watchdog state |
| `/obot2/watchdog/health` | `std_msgs/String` | Overall health classification |
| `/obot2/watchdog/bus_voltage/left` | `std_msgs/Float32` | Left S1 bus voltage |
| `/obot2/watchdog/bus_voltage/right` | `std_msgs/Float32` | Right S1 bus voltage |
| `/obot/watchdog_event` | `std_msgs/String` | Operator notifications |

### Health classification

Verify the health topic reports the appropriate status:

| Status | Meaning |
|---|---|
| `NORMAL` | Both controllers have fresh telemetry and are healthy |
| `RECOVERING` | Recovery, verification, or retry is underway |
| `FAULT` | Controller error or persistent fault |
| `NO_DATA` | CAN communication or required telemetry is unavailable or stale |

A persistent fault takes precedence over other classifications.

Loss of CAN communication or stale required telemetry must not be reported as normal operation.

### Bus-voltage reporting

Verify that both voltage topics publish independently.

When voltage telemetry becomes stale or unavailable, the corresponding `Float32` value should be NaN, not zero.

Monitoring applications must explicitly handle NaN values.

## 3. Controlled hardware verification

Requires separate approval.

Before any powered test, review the actual controller firmware/configuration, CAN bitrate and IDs, battery/BMS response, brake resistor, and robot mechanical support.

Keep the wheels off the ground, have an operator at the physical E-stop, and use an approved, isolated way to simulate faults.

Use an approved, isolated test fixture to simulate faults and CAN communication loss. Do not create a fault by stressing the drive or disconnecting energized power/CAN wiring.

1. Verify healthy heartbeat, `Get_Error`, voltage, and CAN link telemetry for both controllers. Confirm no state-changing CAN frames are sent while both controllers are healthy.

2. Verify that `/obot2/watchdog/health` reports `NORMAL` with healthy controllers and fresh telemetry.

3. With a safe simulated eligible fault on one controller, verify that only that controller receives the recovery sequence and that fresh post-command telemetry is required before success is reported.

4. Verify that a nonrecoverable S1 error is recorded and notified without repeated reset commands. Confirm the health classification becomes `FAULT`.

5. With a controlled test fixture that rejects recovery, verify retries are separated by the configured interval, bounded by `max_recoveries`, and stop after a persistent fault is reported.

6. Simulate CAN communication loss. Verify that:
   - Communication faults are reported.
   - Health changes to `NO_DATA` when required CAN telemetry is unavailable.
   - Recovery commands are withheld while the observed CAN link is not `ERROR-ACTIVE`.

7. Verify stale heartbeat and `Get_Error` telemetry are detected and reported as unavailable data.

8. Verify that stale or unavailable bus-voltage telemetry produces NaN on the corresponding voltage topic.

9. Confirm the laptop popup mechanism receives `/obot/watchdog_event`. Disconnecting the laptop must not affect controller monitoring or recovery.

10. Verify joystick and navigation operation through their existing `twist_mux` inputs with the watchdog running. Confirm the watchdog itself publishes no velocity messages and does not change mux locks.

11. Verify that the diagnostic polling loop staggers left and right `Get_Error` requests by 50 ms and that overlapping right-side requests are handled without blocking the ROS executor.

Stop the test if diagnostics differ from the expected controller response.

Do not treat passing unit tests or a successful clear as proof that an underlying electrical or mechanical fault is resolved.

## 4. Deployment and restart planning

Not performed by this procedure.

After review and explicit deployment approval:

- Rebuild and restart `obot2-watchdog.service` to load watchdog code and YAML.
- If OBot-2's `obot.service` loads `twist_mux_obot2.yaml`, restart `obot.service` to load the removal of its obsolete watchdog input and lock.
- Do not modify the operator joystick service or change other robot services as part of this watchdog update.
- Verify the new diagnostic topics after restart.

No deployment, service restart, or hardware test is implied by completion of this documentation update.
