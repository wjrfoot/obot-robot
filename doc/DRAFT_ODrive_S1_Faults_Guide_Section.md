# DRAFT — ODrive S1 Motor Faults: Regenerative Overvoltage and Recovery

Proposed section for the SprayBot Installation Guide. Not yet merged into the
guide; pending review of the implementation. Numeric limits below are
preliminary engineering settings, not validated safety limits.

## Regenerative braking and back EMF

A spinning motor is also a generator. When the robot decelerates or reverses
quickly, the motor controller returns kinetic energy to the DC bus. If the
supply cannot absorb it (a battery protected by a BMS that blocks charging
current, a long or high-resistance supply path), the bus voltage rises. An
ODrive S1 that sees bus voltage above its configured limit disarms with
`DC_BUS_OVER_VOLTAGE` (0x100) and the motors stop being driven.

## Fault classes

**Isolated transient fault.** A single regenerative overvoltage during rapid
braking or a sudden reversal, where bus voltage returns to its normal range.
This can be recovered automatically once conditions are verified.

**Collision-related faults.** A collision does not necessarily raise the bus
voltage. It more often causes overcurrent, velocity-limit or similar errors.
The watchdog uses the actual ODrive error code: only `DC_BUS_OVER_VOLTAGE` is
eligible for automatic recovery; any other error causes a lockout.

**Persistent unsafe bus voltage.** Voltage that stays high points to a wrong
supply or battery, a braking-resistor wiring or configuration problem, battery
protection interrupting regenerative current, loose or intermittent power
wiring, or a controller hardware fault. Clearing the error does not fix any of
these, so the watchdog never recovers while the voltage is not in its normal
range, and locks out if it does not settle.

**Repeated or periodic faults.** A controller that faults again shortly after
recovering indicates an unresolved electrical or mechanical problem. More than
3 faults, or 3 recovery attempts, within 60 s causes a lockout. Successful
recoveries do not erase this history. **The watchdog must not repeatedly reset
controllers while an electrical fault remains unresolved.**

## Braking resistor and battery protection

A brake resistor on the DC bus dissipates regenerative energy as heat, keeping
the bus below the over-voltage trip. It only works if it is wired, sized and
enabled in the S1 configuration (`DC_BUS_OVER_REGEN_CURRENT` and
`BRAKE_RESISTOR_DISARMED` are related errors). Battery protection circuitry
may block charge current, so regenerated energy has nowhere to go; verify how
the OBot-2 battery/BMS behaves before relying on regeneration.

## Automatic watchdog recovery (OBot-2)

OBot-2: CAN node IDs 61 (left) and 62 (right), 1 Mbit/s. OBot-1: node IDs 0 and
1, 1 Mbit/s. The OBot-1 watchdog is unchanged.

1. On any fault the watchdog immediately inhibits motion (zero-velocity stream
   and a twist_mux lock that persists if the watchdog stops) and notifies the
   laptop.
2. It waits for the operator to release F1 (`/joy` button 4). A fresh joystick
   message showing release is required after the fault; a silent joystick
   never qualifies.
3. If the fault is eligible and the bus voltage is verified normal, it clears
   errors and requests closed-loop control on both controllers.
4. It verifies: both controllers answering, zero active errors, disarm reasons
   cleared, axis state 8 (closed loop) in new heartbeats, normal bus voltage,
   F1 still released, and a zero-velocity command path.
5. Only then is the inhibit released. Driving requires a new F1 press.
6. Failure, persistence or recurrence ends in **lockout**: motion stays
   inhibited, automatic recovery is suspended, a critical notification is
   sent and diagnostics are preserved. F1 does not clear a lockout.

The watchdog supplements, and does not replace, the hardware emergency stop.
Automatic recovery ships disabled; enable it only after the voltage criteria
have been validated for the installed controller settings and battery.

## Diagnostic logging and notifications

Events go to the systemd journal (`journalctl -u obot2-watchdog`) and to
`/home/pi/obot_logs/obot2_watchdog_faults.jsonl`, with time, controller,
CAN state, active errors, disarm reason, axis state, bus voltage,
classification, F1 confirmation, recovery attempts, result, lockout reason and
wheel velocities where available. Notifications are published on
`/obot/watchdog_event` as `[INFO|WARNING|CRITICAL] OBot-2: ...` and shown on
`ros-24` by `obot-watchdog-notify`.

## Manual investigation and recovery after lockout

1. Read the lockout reason (journal, fault log, `/obot2/watchdog/state`).
2. Inspect the cause: supply/battery voltage, brake resistor, wiring and
   connectors, battery protection, motors and mechanics.
3. Correct it; power-cycle the controllers if required.
4. Release F1 and call
   `ros2 service call /obot2/watchdog/manual_reset std_srvs/srv/Trigger`.
   The first call revalidates and asks for confirmation; repeat within 30 s.
5. Confirm closed-loop operation and test with the wheels off the ground
   before driving.
