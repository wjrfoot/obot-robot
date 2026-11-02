# OBot-2 watchdog — bench-test procedure (pre-deployment)

Status: **proposed, not yet executed.** Do not deploy or enable the watchdog
until this procedure is reviewed and approved.

## Ground rules

* Wheels **off the ground** (robot on blocks) for every test, hardware
  E-stop within reach and tested first. The watchdog is not a replacement for
  the E-stop.
* Keep `dry_run: true` and `automatic_recovery_enabled: false` throughout
  (they are the shipped defaults). In this mode the watchdog never transmits
  Clear_Errors or Set_Axis_State. It does transmit Get_Error RTR frames
  (`7A3`, `7C3`) — read-only requests.
* Never create an overvoltage, collision or any real fault on purpose. Faults
  in these tests are simulated by software inputs only.
* Motor power: section A–F and H can be done with motor/battery power
  **off** where noted. Sections G and I need the S1s powered.
* Record every result (PASS/FAIL, observations) in the test log. Any FAIL
  stops the procedure until understood.
* Environment on pi-bot-2 (and ros-24 where noted):

      export ROS_DOMAIN_ID=5 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
      source /opt/ros/jazzy/setup.bash; source ~/obot_ws/install/setup.bash

Where a step says "mux output", observe
`ros2 topic echo /diff_drive_controller/cmd_vel --field twist.linear.x`
(with `--field twist.angular.z` as needed).

---

## A. Pre-checks (no motion)

1. `git status` clean on pi-bot-2, `colcon build` of `obot_watchdog` and
   `obot_bringup` succeeds, `ros2 pkg list | grep twist_mux` present.
2. Check `/home/pi/obot_ws/src/obot_bringup/config/cyclonedds_wifi.xml`
   exists and names pi-bot-2's actual interface; confirm `ros2 topic list`
   from ros-24 shows pi-bot-2 topics.
3. `python3 -m pytest src/obot_watchdog/test -q` passes on the Pi.
4. Confirm the bringup starts with `OBOT_TWIST_MUX_CONFIG` unset exactly as
   before (regression check for OBot-1-style behaviour).
5. Check the twist_mux version (`apt policy ros-jazzy-twist-mux`) and record it:
   the lock semantics below were derived from the twist_mux source.

## B. Watchdog lock on a real twist_mux (no robot, no CAN)

Run the automated check **on a machine with a working twist_mux** (pi-bot-2
with motors/bringup stopped, or any host), in its own isolated domain:

    python3 src/obot_watchdog/test/mux_lock_check.py \
        src/obot_bringup/config/twist_mux_obot2.yaml

(The script sets `ROS_DOMAIN_ID=98` and localhost-only; it starts its own
twist_mux and does not touch the robot.) Expected: `RESULT ALL PASS`:

| # | Check | Expected |
|---|---|---|
| 1 | Fresh mux, no lock message | No output (starts locked) |
| 2 | lock=false, teleop streaming | Teleop passes (positive control) |
| 3 | lock=true + watchdog zero stream | Teleop blocked, zero stream passes |
| 4 | Stop publishing lock while it was false | Teleop blocked after ≤0.5 s |
| 5 | Stop publishing lock while it was true | Still blocked |
| 6 | Navigation input with lock=true | Blocked |
| 7 | Restart mux with teleop streaming, no lock yet | No motion before first lock message |

The script aborts (exit 2) if twist_mux fails to start or the positive control
fails, so "blocked" can never pass vacuously.

Then repeat manually on the real bringup (motor power **off**, controllers
may be inactive; only the mux topics matter):

1. Start bringup with `OBOT_TWIST_MUX_CONFIG=twist_mux_obot2.yaml` and **no
   watchdog**. Publish a teleop command:
   `ros2 topic pub -r 10 /cmd_vel_teleop geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.1}}}"`
   Expect: **nothing** on `/diff_drive_controller/cmd_vel`. (Robot cannot move
   without the watchdog — intended.) Check the mux log for the lock entry
   (`ros2 param get /twist_mux locks.watchdog_lock.timeout` = 0.5, priority 150).
2. Start the watchdog (manually in a terminal, **not** the systemd unit):
   `ros2 launch obot_watchdog obot2_watchdog.launch.py`
   Expect: log line `dry_run=True automatic_recovery_enabled=False
   nodes=61/62 f1_button=4`; `/watchdog_lock` = true (`ros2 topic echo
   /watchdog_lock`); `/cmd_vel_watchdog` zero at ~10 Hz with advancing
   stamps (`ros2 topic hz`, `echo`); the teleop command still blocked.
3. With S1s absent/unpowered the watchdog must stay inhibited (heartbeat
   missing → lockout). That is the expected fail-closed result.

## C. Watchdog process stops

Prerequisite: watchdog running, lock currently **false** (only reachable with
S1s powered and healthy; otherwise use the automated script, test 4).

1. Stream teleop (`ros2 topic pub -r 10 /cmd_vel_teleop ... x: 0.1`); confirm
   it reaches the mux output.
2. Stop the watchdog with `Ctrl-C`. Expect: output stops within ~0.5 s and
   does not resume; `ros2 topic echo /watchdog_lock` is silent.
3. `kill -9` the watchdog process (simulates a crash) with teleop streaming.
   Expect the same.
4. Start it again. Expect: **inhibited** at start (state `NORMAL inhibit=True`,
   released only after closed-loop heartbeats, fresh F1-released `/joy` and a
   zero mux output).
5. Create a persisted-fault restart: with `state_file` containing
   `{"fault_active": true, ...}` (copy a file made by the unit test fixture,
   or write one by hand), start the watchdog. Expect **LOCKOUT**, critical
   notification, no release. Remove the file / use the manual reset to clear.
6. Corrupt the state file (`echo '{x' > state_file`). Expect LOCKOUT
   (`STATE_FILE_UNREADABLE`).

## D. twist_mux restarts

1. Watchdog running, lock false, teleop streaming (as C1).
2. Restart **only twist_mux** (kill the `twist_mux` process; do not restart
   `obot.service`; if bringup is a single launch, run a second standalone
   twist_mux with the same YAML and remap `cmd_vel_out:=/diff_drive_controller/cmd_vel`
   instead, in a test domain).
3. Watch the mux output on `/diff_drive_controller/cmd_vel` with timestamps
   (`ros2 topic echo --field twist` plus `date`, or `ros2 bag record`).
   Expect: **no teleop motion value at any moment** between the restart and
   the first lock=false message; motion resumes only after it, ≤ ~0.2 s later.
4. Repeat 10 times; record maximum "gap" time. Any non-zero output before the
   first lock message is a FAIL.
5. Repeat with the watchdog in the inhibited state: motion must stay blocked
   throughout.

## E. F1 must be released before recovery

Software-only; uses a simulated fault. Provide a way to drive the **logic**
without touching hardware — run the mocked scenarios on the Pi:

    python3 -m pytest src/obot_watchdog/test -q -k "f1 or joystick"

For the live system, only observation is possible without a real fault, so
verify the live inputs the logic depends on:

1. `ros2 topic echo /joy --field buttons` — press and release F1; confirm
   index 4 toggles 1/0 and that `/joy` publishes while F1 is released (the
   joy_node autorepeat/`deadzone` must not suppress release messages). If the
   release message is only sent on change (no autorepeat), record it: the
   watchdog requires a *fresh* message within `joy_timeout_s` (1.0 s), so
   `autorepeat_rate` must be > 1 Hz. Set it if not.
2. With the watchdog running (inhibited, dry run), hold F1 and watch
   `/obot2/watchdog/state`; it must not change. Unplug/turn off the
   controller: state remains inhibited, no recovery attempt, log shows no
   Clear_Errors/Set_Axis_State lines (dry run would log `DRY RUN: would send`).
3. After approval to test recovery logic (see "Later, with approval"), a real
   fault injection is **not** to be done by overvoltage; use only the software
   fault-injection approach agreed beforehand.

## F. Stale joystick commands cannot cause motion

1. Hold F1 and push a stick so the mixer publishes `/cmd_vel_teleop`; confirm
   it reaches the mux output (S1 power off or controller inactive).
2. Turn the PS4 controller off while the stick is held. Expect the mux output
   to go to zero within `joy_timeout_sec` (0.5 s in the teleop launch) and
   stay there. Turn it back on with F1 pressed and the stick held; expect
   motion only per normal mixer rules, never a replay of the old command.
3. Lock engaged (watchdog inhibited): repeat 1. Expect nothing at the mux
   output. Release the lock path only by the watchdog's own logic.
4. Publish a *stale-stamped* command (`header.stamp` 1 minute old) to
   `/cmd_vel_teleop` while the lock is false: record the result. twist_mux
   uses receive time, so this is expected to pass the mux; the protection
   against stale operator commands is the mixer (`joy_timeout`, F1 gating).
   Document the result; it is not a watchdog-level guarantee.
5. After a (simulated) recovery the watchdog releases only with the mux output
   at zero for a fresh sample, and "normal movement requires a new F1
   press": confirm the mixer's `require_enable=true` and `enable_button=4`
   in the OBot-2 launch.

## G. Both S1 bus-voltage readings (S1 powered; passive)

1. Confirm each S1 sends bus voltage cyclically:
   `odrv.can.config.bus_vi_msg_rate_ms > 0` (set via odrivetool on a
   connected S1 *only with approval*; otherwise note the current value).
2. Passively observe (no transmit): `candump -L can0,7B7:7FF,7D7:7FF`.
   Left voltage frame ID 0x7B7 (node 61), right 0x7D7 (node 62). Decode the
   first 4 bytes as little-endian float32.
3. Compare both against a multimeter on the DC bus at each controller. They
   should agree within meter/ADC tolerance; record both readings, the supply
   voltage, and ripple. A frame that is absent, NaN or frozen must show up as
   a stale-voltage lockout in the watchdog (verify by powering one S1 off:
   expect the heartbeat/voltage-stale critical notification, no recovery).
4. Record `dc_max_negative_current`, over-voltage trip
   (`odrv.config.dc_bus_overvoltage_trip_level`) and under-voltage trip,
   brake-resistor settings and battery/BMS regen behaviour. These determine
   `voltage_min_v`, `voltage_normal_max_v`, `voltage_unsafe_v` — they are
   **not** to be guessed; leave them `.nan` until recorded and reviewed.

## H. OBot-2 CAN node IDs 61 and 62 (passive)

1. With only passive tools (`candump`, no `cansend`):
   `candump -L can0 | head -50` for ~10 s.
2. Expect heartbeats on `0x7A1` (node 61, left) and `0x7C1` (node 62,
   right). Expect **no** frames from node IDs 0 and 1 (`0x001`, `0x021`) on
   OBot-2's bus.
3. `ip -details link show can0`: `can state ERROR-ACTIVE`, `bitrate 1000000`.
4. Confirm left/right orientation: node 61 drives the left wheel and 62
   the right (confirmed by the ros2_control hardware interface, URDF
   `node_id` params — the URDF in this repository currently has OBot-1's
   IDs 0 and 1; confirm what pi-bot-2 actually runs). Do not command motion
   to check this.
5. With the watchdog (dry-run) running: `candump -L can0` shows RTR frames
   `7A3#R` / `7C3#R` every 2 s and the replies `7A3`/`7C3` (8 bytes).
   No `7B8`, `7D8`, `7A7`, `7C7` frames may appear (dry run).
6. Run the unit test `test_can_ids_match_obot2_table` on the Pi.

## I. Notifications on ros-24

On ros-24 with `ROS_DOMAIN_ID=5` and the Wi-Fi CycloneDDS config:

1. `bin/obot-watchdog-notify` running (confirm it starts without errors).
2. From pi-bot-2 (or ros-24) publish test events:

       ros2 topic pub --once /obot/watchdog_event std_msgs/msg/String "{data: '[INFO] OBot-2: notification test'}"
       ros2 topic pub --once /obot/watchdog_event std_msgs/msg/String "{data: '[WARNING] OBot-2: notification test'}"
       ros2 topic pub --once /obot/watchdog_event std_msgs/msg/String "{data: '[CRITICAL] OBot-2: notification test'}"
       ros2 topic pub --once /obot/watchdog_event std_msgs/msg/String "{data: 'OBot watchdog: S1 fault detected (untagged OBot-1 style)'}"

   Expect four desktop notifications: normal, "WARNING", "CRITICAL" titles
   and the untagged one as critical with the original title. Record whether a
   first message is lost on discovery (known with `--once`; use `-r 1` for a
   few seconds if so).
3. Stop `obot-watchdog-notify` and confirm the watchdog keeps inhibiting
   normally (notification failure must not affect safety). Block the
   network (disconnect Wi-Fi on the laptop) with the watchdog inhibited:
   inhibit must remain, journal logs the events.
4. Check notifications go only to ros-24 as before; OBot-1's notifier path is
   unchanged.

## Later, with explicit approval only

* Install/enable the systemd unit (start without enabling first), run in
  dry run for a prolonged period, review the journal and
  `~/obot_logs/obot2_watchdog_faults.jsonl`.
* Fill in the voltage thresholds and timeouts from section G.4 (reviewed).
* Only then consider `dry_run:=false` with `automatic_recovery_enabled:=false`
  (manual reset path), and finally automatic recovery after review.

## Exit criteria for approval to deploy

* Section B script ALL PASS on the robot's twist_mux version; manual B1–B2
  as described.
* C, D: no motion in any of the failure/restart cases, over 10 restarts.
* E–F recorded, with `/joy` autorepeat confirmed.
* G: both voltage readings agree with the meter; thresholds derived from
  recorded controller configuration.
* H: node IDs, orientation, bitrate confirmed; no mutating CAN frames in dry
  run.
* I: all four notification types observed.
