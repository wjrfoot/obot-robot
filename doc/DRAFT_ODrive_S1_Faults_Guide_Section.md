# DRAFT — ODrive S1 motor faults and watchdog recovery

Proposed section for the SprayBot Installation Guide. This describes the
OBot-2 watchdog implementation; it is not evidence that recovery has been
validated on the robot's installed firmware or battery system.

## Regenerative braking and bus overvoltage

A spinning motor acts as a generator. During rapid deceleration or reversal,
regenerated energy raises the DC bus voltage. An S1 may disarm with
`DC_BUS_OVER_VOLTAGE` (`0x100`) if the supply cannot absorb this energy.
Battery/BMS behavior, brake-resistor configuration, wiring, and controller
limits must be investigated if this fault recurs.

## OBot-2 watchdog behavior

The watchdog monitors the left S1 at CAN node ID 61 and right S1 at ID 62 on
`can0` at 1 Mbps. It is independent of joystick and velocity control: it does
not subscribe to `/joy`, inspect F1, publish velocity commands, or operate a
`twist_mux` lock. Normal teleoperation and navigation routing are unchanged.

For each controller, the watchdog records heartbeat errors, `Get_Error`
active errors and disarm reason, axis state, bus voltage when fresh, CAN link
state, controller ID, and timestamp. Fault events are logged and published on
`/obot/watchdog_event` for the existing laptop popup notifier.

Automatic recovery is enabled but bounded. Only `DC_BUS_OVER_VOLTAGE` is
eligible for automatic error clearing; other active error bits are reported
without issuing `Clear_Errors`. The recovery sequence is:

1. Send `Clear_Errors` for the affected controller.
2. After the configured command gap, request `CLOSED_LOOP_CONTROL` (state 8).
3. Verify with fresh post-command heartbeat and `Get_Error` telemetry that
   the controller is in state 8 and has no active or disarm errors.
4. Retry at the configured interval, up to three attempts, then report a
   persistent fault and stop issuing recovery commands.

The watchdog does not restart `ros2_control` or change the other S1 when one
controller faults. It does not impose voltage thresholds; bus voltage is
recorded as diagnostic information. Validate ODrive error bits, CAN IDs,
bitrate, fault-clearing behavior, and recovery on a controlled bench before
relying on automatic recovery.

## Notifications and investigation

The robot logs to the systemd journal (`journalctl -u obot2-watchdog`) and
appends records to
`/home/pi/obot_logs/obot2_watchdog_faults.jsonl`. The existing operator-laptop
process `obot-watchdog-notify` displays messages from `/obot/watchdog_event`.

On a fault, first inspect the event record and the S1 diagnostics. Check the
battery/BMS, brake resistor, supply wiring, controller configuration, motor,
and mechanics as appropriate. Repeated faults require investigation; do not
treat a successful reset as proof that the underlying cause is resolved.

The watchdog is not an emergency stop. The physical E-stop remains the
emergency-stop mechanism.
