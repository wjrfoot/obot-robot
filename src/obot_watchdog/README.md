# obot_watchdog — OBot-2 ODrive S1 fault watchdog

Separate from the OBot-1 `bin/obot-watchdog` (which is unchanged). Maintained
only in this repository; the laptop only runs the existing
`obot-watchdog-notify` (in `obot-laptop`), which subscribes to
`/obot/watchdog_event`.

**Status: implemented and tested with mocked CAN/ROS only. Not deployed.**
Ships in the safest configuration: `dry_run: true`,
`automatic_recovery_enabled: false`, voltage thresholds and recovery timeouts
unset (`.nan`).

## OBot-2 configuration

| Item | Value |
|---|---|
| CAN | `can0`, 1,000,000 bit/s (bitrate mismatch is treated as a CAN fault) |
| S1 node IDs | left 61, right 62 (OBot-1 uses 0 and 1) |
| Deadman | `/joy` `buttons[4]` (PS4 F1) |
| ROS | Jazzy, `ROS_DOMAIN_ID=5`, `rmw_cyclonedds_cpp` |

| Operation | Left | Right |
|---|---|---|
| Heartbeat | 0x7A1 | 0x7C1 |
| Get_Error (RTR) | `7A3#R` | `7C3#R` |
| Clear_Errors | `7B8#` | `7D8#` |
| Set_Axis_State closed loop | `7A7#08000000` | `7C7#08000000` |

Frame layouts were checked against the ODrive CAN protocol documentation
(Clear_Errors has an optional Identify byte that is omitted, as in the
OBot-1 `obot-drive-recover` script; confirm on the installed firmware).
Transmission is never assumed to mean acceptance: success is judged only from
heartbeat axis state/error and fresh Get_Error replies after the commands.

## Behaviour

States: NORMAL, FAULT_DETECTED, WAIT_FOR_F1_RELEASE, RECOVERING, VERIFYING,
LOCKOUT (see `obot_watchdog/core.py`; pure logic, ticked at 10 Hz, no sleeps).

* The robot starts **inhibited** and is released only after both S1 report
  closed loop with fresh data, F1 is freshly confirmed released, and the mux
  output is observed at zero.
* A fault inhibits motion in the same tick and publishes a notification.
* Only `DC_BUS_OVER_VOLTAGE` (`recoverable_error_mask`) is eligible. Other
  S1 errors, and loss of CAN/heartbeat/Get_Error/voltage telemetry, go straight
  to LOCKOUT.
* Recovery needs a **fresh** `/joy` message with F1 released after the fault;
  silence never qualifies. Then: voltage NORMAL (else wait up to
  `voltage_settle_timeout_s`, then lockout), no disqualifying fault, history
  within limits, then Clear_Errors → (0.5 s) → closed-loop request on both.
* Verification: both heartbeats and Get_Error replies newer than the command,
  zero errors/disarm, state 8, voltage NORMAL, CAN ERROR-ACTIVE, F1 still
  released, mux output observed at zero. Failure retries within the limit.
* Limits (preliminary): more than 3 faults, or 3 recovery attempts, within
  60 s → LOCKOUT. Successful recoveries do not erase history.
* LOCKOUT is persisted in `state_file`; a watchdog restart during a fault
  restores LOCKOUT. F1 never clears it. Manual reset is a two-step service:

      ros2 service call /obot2/watchdog/manual_reset std_srvs/srv/Trigger

  The first call revalidates (F1 released, no disqualifying fault, voltage) and
  asks for confirmation; the second call within 30 s clears the lockout and
  history. If the drives are healthy the robot re-arms like a fresh start.
* The watchdog never restarts `obot.service` or cycles
  `diff_drive_controller`.

`dry_run: true` suppresses only state-changing CAN frames (Clear_Errors,
Set_Axis_State); Get_Error requests, notifications, logging and the motion
inhibit are still active. `automatic_recovery_enabled: false` makes every
eligible fault end in LOCKOUT (reason `AUTOMATIC_RECOVERY_DISABLED`) so an
operator investigates; a manual reset can then authorise one operator-directed
recovery.

## Motion interlock (needs review before enabling recovery)

`config/twist_mux_obot2.yaml` (in `obot_bringup`) adds:

* `/cmd_vel_watchdog` (TwistStamped, priority 200, timeout 0.5 s) — zero
  velocity at 10 Hz with fresh stamps while inhibited.
* A twist_mux **lock** `/watchdog_lock` (Bool, priority 150, **timeout 0.5 s**).
  Locks mask all topics below their priority. In the twist_mux source a lock
  is LOCKED when it has expired *or* its last Bool is true, and a lock that has
  never received a message counts as expired. A positive timeout is therefore
  fail-closed: twist_mux starts locked, and re-locks 0.5 s after the watchdog
  stops publishing (it publishes at 10 Hz). `timeout: 0.0` would never expire
  and would start unlocked, so it must not be used.
* Consequence: with this config the robot cannot be driven unless the
  watchdog is running. To run OBot-2 without it, deliberately start bringup
  with the original `twist_mux.yaml` (leave `OBOT_TWIST_MUX_CONFIG` unset).
* twist_mux timeouts use message *receive* time, not the header stamp, so a
  republished stale command is not rejected by the mux itself; staleness is
  handled by the lock, the mixer's `/joy` timeout and F1 gating.

See `doc/BENCH_TEST_PROCEDURE.md` (repository root `doc/`) before deployment.

Selected per robot: `OBOT_TWIST_MUX_CONFIG=twist_mux_obot2.yaml` in the
environment of `service_scripts/start_obot.sh` (unset = original
`twist_mux.yaml`, i.e. OBot-1 unchanged).

## Log, state and notifications

* ROS log / journal: `journalctl -u obot2-watchdog`
* `/home/pi/obot_logs/obot2_watchdog_faults.jsonl` (fsync per record: time,
  robot, per-controller errors/disarm/axis state/voltage, CAN state, class,
  attempts, F1 confirmation, result, lockout reason, commanded/measured
  velocity)
* `/obot/watchdog_event` text: `[INFO|WARNING|CRITICAL] OBot-2: ...`
  (the laptop notifier maps the tag to urgency; untagged OBot-1 events behave
  as before). Repeated identical non-critical events are rate limited.
* `/obot2/watchdog/state` (String, 1 Hz)

## Build and test (no hardware)

    cd ~/Workspaces/obot-robot && source /opt/ros/jazzy/setup.bash
    colcon build --packages-select obot_watchdog --symlink-install
    cd src/obot_watchdog && python3 -m pytest test -q

## Deployment (requires approval; not done)

1. On pi-bot-2 pull the repo, `colcon build`, and confirm
   `~/obot_ws/src/obot_bringup/config/cyclonedds_wifi.xml` exists and names
   pi-bot-2's own interface (the unit refuses to start otherwise).
2. Install `twist_mux` if absent; set `OBOT_TWIST_MUX_CONFIG` for the OBot-2
   bringup and restart bringup in a planned window (wheels off ground).
3. `sudo cp systemd/obot2-watchdog.service /etc/systemd/system/`,
   `sudo systemctl daemon-reload`, then start (do not enable yet) and watch
   the journal in dry-run.
4. On ros-24 run `bin/obot-watchdog-notify` and confirm a test event.
5. Before `dry_run:=false` / `automatic_recovery_enabled:=true`, validate the
   items below.

## Unresolved items

* Voltage thresholds and `verify_timeout_s` / `voltage_settle_timeout_s` are
  unset: derive from the verified S1 over-voltage trip, brake-resistor
  settings and battery/BMS behaviour.
* Interlock design (fail-closed lock timeout, startup and restart behaviour)
  follows the twist_mux source but has not been run against a real twist_mux;
  bench-test with wheels lifted.
  A hardware E-stop remains the primary safety.
* 60 s window / 3 recoveries are preliminary.
* twist_mux restart: by the source analysis the restarted mux starts locked
  until the watchdog's first lock message; confirm on the robot's twist_mux
  version (`test/mux_lock_check.py`, bench procedure section B).
* The robot URDF in this repo still has OBot-1 node IDs (0/1); the OBot-2
  `ros2_control` configuration with 61/62 must be confirmed on pi-bot-2.
* Clear_Errors payload (Identify byte) and S1 firmware error bit values need
  confirming on the installed firmware.
