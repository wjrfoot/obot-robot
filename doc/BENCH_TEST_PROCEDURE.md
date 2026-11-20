# OBot-2 watchdog verification procedure

Status: **test plan only; not executed on hardware.** Unit tests use mocked ROS
timers and CAN I/O and do not establish hardware safety. Do not deploy or
exercise fault recovery on a robot until the code and this procedure have
been reviewed and the E-stop and bench setup are confirmed.

## 1. Software checks (no hardware)

From the repository root in a sourced ROS 2 Jazzy environment:

```sh
colcon build --packages-select obot_watchdog --symlink-install
python3 -m pytest src/obot_watchdog/test -q
```

Confirm the tests cover per-controller fault detection, diagnostic records,
`Clear_Errors`, state-8 requests, fresh telemetry verification, bounded
retries, CAN communication faults, event notifications, and the 50 ms
left/right `Get_Error` stagger.

Inspect the configured command values and limits in
`src/obot_watchdog/config/obot2_watchdog.yaml`. Recovery is enabled in the
current configuration, but only the configured recoverable error mask
(`0x100`, `DC_BUS_OVER_VOLTAGE`) is automatically cleared. Confirm that the
installed ODrive S1 firmware uses the expected error-bit definitions before
deployment.

## 2. Review the communication and routing configuration

Confirm OBot-2 uses CAN node IDs 61 (left) and 62 (right), `can0`, and
1 Mbps. Confirm that `twist_mux_obot2.yaml` retains both `/cmd_vel_teleop`
and `/cmd_vel_nav` and contains no watchdog velocity topic or lock. The
watchdog must not subscribe to `/joy` or publish velocity commands.

The node logs fault diagnostics to
`/home/pi/obot_logs/obot2_watchdog_faults.jsonl` and publishes tagged
notifications on `/obot/watchdog_event`; the existing operator-laptop
`obot-watchdog-notify` process remains responsible for popup display.

## 3. Controlled hardware verification (requires separate approval)

Before any powered test, review the actual controller firmware/configuration,
CAN bitrate and IDs, battery/BMS response, brake resistor, and the robot's
mechanical support. Keep the wheels off the ground, have an operator at the
physical E-stop, and use an approved, isolated way to simulate faults. Do not
create a fault by stressing the drive or disconnecting energized power/CAN
wiring.

1. Verify healthy heartbeat, `Get_Error`, voltage, and CAN link telemetry for
   both controllers. Confirm no state-changing CAN frames are sent while both
   controllers are healthy.
2. With a safe simulated eligible fault on one controller, verify that only
   that controller receives the recovery sequence and that fresh post-command
   telemetry is required before success is reported.
3. Verify that a nonrecoverable S1 error is recorded and notified without
   repeated reset commands.
4. With a controlled test fixture that rejects recovery, verify retries are
   separated by the configured interval, bounded by `max_recoveries`, and
   stop after a persistent fault is reported.
5. Simulate CAN/heartbeat/Get_Error communication loss. Verify that it is
   reported, and that recovery commands are withheld while the observed CAN
   link is not `ERROR-ACTIVE`.
6. Confirm the laptop popup mechanism receives `/obot/watchdog_event`.
   Disconnecting the laptop must not affect controller monitoring or
   recovery.
7. Verify joystick and navigation operation through their existing
   `twist_mux` inputs with the watchdog running. Confirm the watchdog itself
   publishes no velocity messages and does not change mux locks.

Stop the test if diagnostics differ from the expected controller response.
Do not treat passing unit tests or a successful clear as proof that an
underlying electrical or mechanical fault is resolved.

## 4. Deployment/restart planning (not performed)

After review and explicit deployment approval:

* Rebuild and restart `obot2-watchdog.service` to load watchdog code and YAML.
* If OBot-2's `obot.service` loads `twist_mux_obot2.yaml`, restart
  `obot.service` to load the removal of its obsolete watchdog input and lock.
  The joystick and navigation inputs remain configured.
* Do not modify the operator joystick service or change any other robot
  services as part of this watchdog update.

No deployment, service restart, or hardware test has been performed by this
change.
