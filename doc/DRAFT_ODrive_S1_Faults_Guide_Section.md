# DRAFT — ODrive S1 motor faults and watchdog recovery

Proposed section for the SprayBot Installation Guide. This describes the OBot-2 watchdog implementation; it is not evidence that recovery has been validated on the robot's installed firmware or battery system.

## Regenerative braking and bus overvoltage

A spinning motor acts as a generator. During rapid deceleration or reversal, regenerated energy raises the DC bus voltage. An S1 may disarm with `DC_BUS_OVER_VOLTAGE` (`0x100`) if the supply cannot absorb this energy.

Battery/BMS behavior, brake-resistor configuration, wiring, and controller limits must be investigated if this fault recurs.

Battery depletion or an abrupt battery-protection shutdown can also interrupt the Raspberry Pi and motor-controller power. Such an event should be investigated separately from an ordinary recoverable motor fault.

## OBot-2 watchdog behavior

The watchdog monitors the left S1 at CAN node ID 61 and right S1 at ID 62 on `can0` at 1 Mbps.

It is independent of joystick and velocity control: it does not subscribe to `/joy`, publish velocity commands, or operate a `twist_mux` lock. Normal teleoperation and navigation routing are unchanged.

For each controller, the watchdog records heartbeat errors, `Get_Error` active errors and disarm reason, axis state, bus voltage when fresh, CAN link state, controller ID, and timestamp.

Fault events are logged and published on `/obot/watchdog_event` for the existing laptop popup notifier.

Automatic recovery is enabled but bounded. Only `DC_BUS_OVER_VOLTAGE` is eligible for automatic error clearing; other active error bits are reported without issuing `Clear_Errors`.

The recovery sequence is:

1. Send `Clear_Errors` for the affected controller.
2. After the configured command gap, request `CLOSED_LOOP_CONTROL` (state 8).
3. Verify with fresh post-command heartbeat and `Get_Error` telemetry that the controller is in state 8 and has no active or disarm errors.
4. Retry at the configured interval, up to three attempts, then report a persistent fault and stop issuing recovery commands.

The watchdog does not restart `ros2_control` or change the other S1 when one controller faults. It does not impose voltage thresholds; bus voltage is recorded as diagnostic information.

## Diagnostic status topics

The watchdog publishes diagnostic information for external monitoring:

| Topic | Purpose |
|---|---|
| `/obot2/watchdog/state` | Internal watchdog state |
| `/obot2/watchdog/health` | Overall health classification |
| `/obot2/watchdog/bus_voltage/left` | Left S1 DC bus voltage |
| `/obot2/watchdog/bus_voltage/right` | Right S1 DC bus voltage |

The health topic publishes one of four classifications:

- `NORMAL`: Both controllers have fresh telemetry, the CAN link is healthy, and both controllers are in closed-loop operation without reported errors.
- `RECOVERING`: At least one controller is undergoing recovery, verification, or retry waiting.
- `FAULT`: A persistent fault or controller error has been detected.
- `NO_DATA`: CAN communication or required controller telemetry is unavailable or stale.

A persistent fault takes precedence over other classifications. Loss of CAN communication or stale required telemetry is reported as `NO_DATA`, not as healthy operation.

Bus voltage is published independently for each controller as `Float32`. A NaN value indicates that voltage telemetry is stale or unavailable.

These topics provide diagnostic information for monitoring tools and operator displays. They do not control robot motion.

## Diagnostic polling

The diagnostic polling loop transmits the left `Get_Error` immediately and stages the right request for 50 ms later using a non-blocking ROS timer.

A 10 ms timer checks the deadline. If a second right-side diagnostic arrives while one is still queued, the outstanding request is sent immediately and the overlap is logged before the new request is scheduled.

Shutdown cancels the timer and clears its pending request.

CAN link errors, missing/stale heartbeats, and missing `Get_Error` replies are reported as communication faults. Mutating CAN commands are not issued while the observed CAN link is not `ERROR-ACTIVE`.

## Notifications and investigation

The robot logs to the systemd journal (`journalctl -u obot2-watchdog`) and appends records to:

`/home/pi/obot_logs/obot2_watchdog_faults.jsonl`

The existing operator-laptop process `obot-watchdog-notify` displays messages from `/obot/watchdog_event`.

On a fault, first inspect the event record and the S1 diagnostics. Check the battery/BMS, brake resistor, supply wiring, controller configuration, motor, and mechanics as appropriate.

Repeated faults require investigation; do not treat a successful reset as proof that the underlying cause is resolved.

The watchdog is not an emergency stop. The physical E-stop remains the emergency-stop mechanism.
