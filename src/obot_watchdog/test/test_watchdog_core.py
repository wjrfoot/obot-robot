"""Hardware-free tests: mocked CAN telemetry, joystick and clock.

Voltage numbers used here are TEST VALUES ONLY and are not recommended
OBot-2 settings (the real thresholds are intentionally unset).
"""

import json
import math
import os

import pytest

from obot_watchdog import odrive_can as oc
from obot_watchdog.can_io import CanTelemetry, make_nodes, parse_can_link
from obot_watchdog.core import (
    CRITICAL, Config, Core, Frame, INFO, NodeTel, Notify, Snapshot, State,
    WARNING,
)

OV = oc.ERR_DC_BUS_OVER_VOLTAGE
OC = 0x00000400  # DC_BUS_OVER_CURRENT
DT = 0.1


def make_cfg(**kw):
    base = dict(
        voltage_min_v=20.0, voltage_normal_max_v=28.0, voltage_unsafe_v=32.0,
        verify_timeout_s=5.0, voltage_settle_timeout_s=3.0,
        automatic_recovery_enabled=True)
    base.update(kw)
    return Config(**base)


class Drive:
    def __init__(self):
        self.error = 0
        self.state = oc.AXIS_STATE_CLOSED_LOOP
        self.v = 25.0
        self.responsive = True
        self.answers_get_error = True
        self.answers_voltage = True
        self.accepts_clear = True
        self.accepts_closed_loop = True
        self.clears = 0


class Sim:
    def __init__(self, cfg=None, boot=True):
        self.cfg = cfg or make_cfg()
        self.t = 1000.0
        self.core = Core(self.cfg, now0=self.t)
        self.left, self.right = Drive(), Drive()
        self.ltel = NodeTel('left', 61)
        self.rtel = NodeTel('right', 62)
        self.joy_on = True
        self.f1 = False
        self.joy_time = None
        self.cmd_safe = True
        self.can_state = 'ERROR-ACTIVE'
        self.can_seq = 0
        self.can_time = None
        self.next_can = self.t
        self.frames = []
        self.notes = []
        self.logs = []
        self.pending_ge = []
        self.pending_voltage = []
        if boot:
            self.advance(self.cfg.startup_grace_s + 2)
            assert not self.core.inhibit

    def drives(self):
        return ((self.left, self.ltel), (self.right, self.rtel))

    def advance(self, seconds):
        for _ in range(int(round(seconds / DT))):
            self.t += DT
            if self.t >= self.next_can:
                self.can_seq += 1
                self.can_time = self.t
                self.next_can += self.cfg.monitor_interval
            for drive, tel in self.drives():
                if drive.responsive:
                    tel.hb_time = self.t
                    tel.hb_error = drive.error
                    tel.hb_state = drive.state
            for drive, tel in self.pending_ge:
                if drive.responsive and drive.answers_get_error:
                    tel.ge_time = self.t
                    tel.ge_active = drive.error
                    tel.ge_disarm = 0
            self.pending_ge = []
            for drive, tel in self.pending_voltage:
                if drive.responsive and drive.answers_voltage:
                    tel.v_time = self.t
                    tel.v = drive.v
            self.pending_voltage = []
            if self.joy_on:
                self.joy_time = self.t
            snap = Snapshot(
                left=NodeTel(**vars(self.ltel)),
                right=NodeTel(**vars(self.rtel)),
                can_state=self.can_state, can_time=self.can_time,
                can_seq=self.can_seq, joy_time=self.joy_time,
                f1_pressed=self.f1, cmd_path_safe=self.cmd_safe)
            for act in self.core.tick(self.t, snap):
                self._handle(act)

    def _handle(self, act):
        if isinstance(act, Notify):
            self.notes.append(act)
        elif isinstance(act, Frame):
            self.frames.append(act)
            drive, tel = (self.left, self.ltel) if act.node == 'left' \
                else (self.right, self.rtel)
            if act.label == 'get_error':
                self.pending_ge.append((drive, tel))
            elif act.label == 'get_voltage':
                self.pending_voltage.append((drive, tel))
            elif act.label == 'clear_errors':
                drive.clears += 1
                if drive.accepts_clear:
                    drive.error = 0
            elif act.label == 'set_closed_loop':
                if drive.accepts_closed_loop and drive.error == 0:
                    drive.state = oc.AXIS_STATE_CLOSED_LOOP
        else:
            if hasattr(act, 'record'):
                self.logs.append(act.record)

    def mutating(self):
        return [f for f in self.frames if f.mutating]

    def fault_ov(self, drive=None):
        d = drive or self.left
        d.error = OV
        d.state = 1  # IDLE
        d.v = 31.0

    def levels(self):
        return [n.level for n in self.notes]


# ---- 1. normal operation -------------------------------------------------
def test_normal_operation():
    sim = Sim()
    sim.advance(30)
    assert sim.core.state == State.NORMAL
    assert not sim.core.inhibit
    assert sim.notes == []
    assert sim.mutating() == []
    assert any(f.label == 'get_error' for f in sim.frames)


def test_startup_stays_inhibited_until_f1_released_and_cmd_path_safe():
    sim = Sim(boot=False)
    sim.f1 = True
    sim.advance(10)
    assert sim.core.inhibit
    sim.f1 = False
    sim.cmd_safe = False
    sim.advance(5)
    assert sim.core.inhibit
    sim.cmd_safe = True
    sim.advance(1)
    assert not sim.core.inhibit


# ---- 2. single transient overvoltage ------------------------------------
def test_single_transient_overvoltage_recovers():
    sim = Sim()
    sim.f1 = True
    sim.fault_ov()
    sim.advance(DT * 3)
    assert sim.core.inhibit
    assert sim.core.state == State.WAIT_FOR_F1_RELEASE
    first = sim.notes[0]
    assert first.level == WARNING
    assert 'OBot-2' in first.text and 'Left' in first.text
    assert 'DC_BUS_OVER_VOLTAGE' in first.text
    sim.left.v = 25.0
    sim.f1 = False
    sim.advance(10)
    assert sim.core.state == State.NORMAL
    assert not sim.core.inhibit
    assert sim.notes[-1].level == INFO
    assert 'closed-loop' in sim.notes[-1].text.lower()
    labels = [f.label for f in sim.mutating()]
    assert labels.count('clear_errors') == 2
    assert labels.count('set_closed_loop') == 2
    assert sim.core.fault_history  # history survives a successful recovery
    assert sim.core.f1_release_confirmed


# ---- 3 / 4. F1 held then released ---------------------------------------
def test_f1_held_waits_indefinitely_and_stays_inhibited():
    sim = Sim()
    sim.f1 = True
    sim.fault_ov()
    sim.left.v = 25.0
    sim.advance(300)
    assert sim.core.state == State.WAIT_FOR_F1_RELEASE
    assert sim.core.inhibit
    assert sim.mutating() == []


def test_f1_release_after_fault_starts_recovery():
    sim = Sim()
    sim.f1 = True
    sim.fault_ov()
    sim.left.v = 25.0
    sim.advance(5)
    assert sim.mutating() == []
    sim.f1 = False
    sim.advance(1)
    assert sim.core.state in (State.RECOVERING, State.VERIFYING)
    assert sim.core.f1_release_confirmed


# ---- 5. joystick loss ---------------------------------------------------
def test_joystick_loss_does_not_trigger_recovery():
    sim = Sim()
    sim.f1 = False
    sim.fault_ov()
    sim.joy_on = False  # last message (F1 released) predates the fault
    sim.left.v = 25.0
    sim.advance(120)
    assert sim.core.state == State.WAIT_FOR_F1_RELEASE
    assert sim.core.inhibit
    assert sim.mutating() == []


def test_joystick_loss_during_recovery_returns_to_wait():
    sim = Sim()
    sim.fault_ov()
    sim.left.v = 25.0
    sim.left.accepts_closed_loop = False
    sim.advance(1.5)
    assert sim.core.state == State.VERIFYING
    sim.joy_on = False
    sim.advance(3)
    assert sim.core.state == State.WAIT_FOR_F1_RELEASE
    assert sim.core.inhibit


# ---- 6. persistent unsafe voltage ---------------------------------------
def test_persistent_unsafe_voltage_locks_out_without_clearing():
    sim = Sim()
    sim.fault_ov()
    sim.left.v = 33.0
    sim.advance(30)
    assert sim.core.state == State.LOCKOUT
    assert sim.core.lockout_reason == 'PERSISTENT_UNSAFE_BUS_VOLTAGE'
    assert sim.core.fault_class == 'PERSISTENT_VOLTAGE'
    assert sim.mutating() == []
    assert CRITICAL in sim.levels()
    assert sim.core.inhibit


def test_voltage_telemetry_loss_is_not_treated_as_safe():
    sim = Sim()
    sim.left.responsive = False
    sim.advance(3)
    assert sim.core.inhibit
    assert sim.core.state == State.LOCKOUT


# ---- 7. repeated faults --------------------------------------------------
def test_repeated_faults_within_window_lock_out():
    sim = Sim()
    for _ in range(3):
        sim.fault_ov()
        sim.left.v = 25.0
        sim.advance(8)
        assert sim.core.state == State.NORMAL
    sim.fault_ov()
    sim.left.v = 25.0
    sim.advance(1)
    assert sim.core.state == State.LOCKOUT
    assert sim.core.lockout_reason == 'REPEATED_FAULTS_WITHIN_WINDOW'
    assert sim.core.fault_class == 'REPEATED'
    assert sim.left.clears == 3
    assert sim.core.inhibit


def test_old_faults_outside_window_do_not_count():
    sim = Sim()
    sim.fault_ov()
    sim.left.v = 25.0
    sim.advance(8)
    sim.advance(61)
    assert sim.core._count(sim.core.fault_history) == 0


# ---- 8. CAN interface failure -------------------------------------------
def test_can_interface_failure_locks_out():
    sim = Sim()
    sim.can_state = 'BUS-OFF'
    sim.advance(6)
    assert sim.core.state == State.LOCKOUT
    assert sim.core.fault_class == 'COMMUNICATION_LOSS'
    assert sim.core.inhibit
    assert sim.mutating() == []


def test_can_state_poller_silent_is_unknown_not_healthy():
    sim = Sim()
    sim.next_can = 1e12  # poller thread died: no new samples
    sim.advance(10)
    assert sim.core.state == State.LOCKOUT


def test_bitrate_mismatch_reported():
    text = 'can state ERROR-ACTIVE\n bitrate 500000 sample-point 0.875'
    assert parse_can_link(text, 1000000).startswith('BITRATE_MISMATCH')
    text = 'can state ERROR-ACTIVE\n bitrate 1000000 sample-point 0.875'
    assert parse_can_link(text, 1000000) == 'ERROR-ACTIVE'
    assert parse_can_link('garbage', 1000000) == 'UNKNOWN'


# ---- 9. missing S1 responses --------------------------------------------
def test_missing_heartbeat_locks_out():
    sim = Sim()
    sim.right.responsive = False
    sim.advance(3)
    assert sim.core.state == State.LOCKOUT
    assert 'Right' in sim.notes[0].text


def test_unanswered_get_error_locks_out():
    sim = Sim()
    sim.right.answers_get_error = False
    sim.advance(10)
    assert sim.core.state == State.LOCKOUT
    assert sim.core.inhibit


# ---- 10. failure to enter closed loop -----------------------------------
def test_closed_loop_failure_retries_then_locks_out():
    sim = Sim()
    sim.fault_ov()
    sim.left.v = 25.0
    sim.left.accepts_closed_loop = False
    for _ in range(300):
        sim.advance(1)
        assert sim.core.inhibit
        if sim.core.state == State.LOCKOUT:
            break
    assert sim.core.state == State.LOCKOUT
    assert sim.core.lockout_reason == 'RECOVERY_FAILED_AT_LIMIT'
    assert sim.left.clears == 3
    assert 'Recovery unsuccessful' in ' '.join(n.text for n in sim.notes)


# ---- 11. notification failure (core is independent of delivery) ---------
def test_inhibit_does_not_depend_on_notifications():
    sim = Sim()
    sim.notes = None  # any delivery attempt in the harness would raise
    sim._handle = lambda act: None  # the notification path is dropped
    sim.fault_ov()
    sim.advance(DT * 2)
    assert sim.core.inhibit


# ---- 12. watchdog process failure ---------------------------------------
def test_restart_during_fault_fails_closed():
    core = Core(make_cfg(), now0=0.0)
    acts = core.restore(5.0, fault_active=True, lockout_reason=None)
    assert core.state == State.LOCKOUT and core.inhibit
    assert any(isinstance(a, Notify) and a.level == CRITICAL for a in acts)


def test_state_store_round_trip_and_unreadable(tmp_path):
    from obot_watchdog.node import StateStore
    store = StateStore(str(tmp_path / 'sub' / 'state.json'))
    assert store.load() == (False, None)
    store.save(True, 'X')
    assert store.load() == (True, 'X')
    with open(store.path, 'w') as fh:
        fh.write('{not json')
    assert store.load()[0] is True


def test_twist_mux_lock_is_fail_closed_and_outranks_inputs():
    import yaml
    path = os.path.join(
        os.path.dirname(__file__), '..', '..', 'obot_bringup', 'config',
        'twist_mux_obot2.yaml')
    cfg = yaml.safe_load(open(path))['twist_mux']['ros__parameters']
    lock = cfg['locks']['watchdog_lock']
    topics = cfg['topics']
    # timeout 0 never expires and starts unlocked; positive is fail-closed
    assert lock['timeout'] > 0.0
    assert topics['watchdog']['priority'] == 200
    assert topics['watchdog']['topic'] == '/cmd_vel_watchdog'
    assert topics['joystick']['priority'] < lock['priority'] \
        < topics['watchdog']['priority']
    assert topics['navigation']['priority'] < lock['priority']


# ---- 13. stale velocity commands ----------------------------------------
def test_unsafe_command_path_blocks_release_after_recovery():
    sim = Sim()
    sim.fault_ov()
    sim.left.v = 25.0
    sim.cmd_safe = False  # stale/non-zero command still reaching the mux
    sim.advance(15)
    assert sim.core.inhibit
    assert sim.core.state != State.NORMAL
    sim.cmd_safe = True
    sim.advance(10)
    assert sim.core.state == State.NORMAL
    assert not sim.core.inhibit


# ---- 14. automatic recovery disabled ------------------------------------
def test_automatic_recovery_disabled_by_default():
    sim = Sim(Config(), boot=False)
    sim.advance(7)
    assert not sim.core.inhibit
    sim.fault_ov()
    sim.advance(10)
    assert sim.core.state == State.LOCKOUT
    assert sim.core.lockout_reason == 'AUTOMATIC_RECOVERY_DISABLED'
    assert sim.mutating() == []
    assert CRITICAL in sim.levels()


def test_unvalidated_criteria_refuse_recovery():
    sim = Sim(Config(automatic_recovery_enabled=True), boot=False)
    sim.advance(7)
    sim.fault_ov()
    sim.advance(5)
    assert sim.core.state == State.LOCKOUT
    assert sim.core.lockout_reason == 'RECOVERY_CRITERIA_NOT_CONFIGURED'
    assert sim.mutating() == []


def test_nonrecoverable_error_locks_out():
    sim = Sim()
    sim.left.error = OC
    sim.left.state = 1
    sim.advance(1)
    assert sim.core.state == State.LOCKOUT
    assert sim.core.fault_class == 'NONRECOVERABLE'
    assert sim.mutating() == []


# ---- 15. recovery lockout -----------------------------------------------
def test_lockout_not_cleared_by_f1_and_requires_two_step_reset():
    sim = Sim()
    sim.left.error = OC
    sim.left.state = 1
    sim.advance(1)
    assert sim.core.state == State.LOCKOUT
    sim.f1 = True
    sim.advance(1)
    sim.f1 = False
    sim.advance(20)
    assert sim.core.state == State.LOCKOUT and sim.core.inhibit

    def snap():
        return Snapshot(
            left=NodeTel(**vars(sim.ltel)), right=NodeTel(**vars(sim.rtel)),
            can_state=sim.can_state, can_time=sim.can_time,
            can_seq=sim.can_seq, joy_time=sim.joy_time, f1_pressed=sim.f1,
            cmd_path_safe=sim.cmd_safe)

    ok, msg = sim.core.request_manual_reset(sim.t, snap())
    assert not ok and 'Revalidation failed' in msg  # fault still present

    # Operator investigates, repairs and power-cycles the controller.
    sim.left.error = 0
    sim.left.state = oc.AXIS_STATE_CLOSED_LOOP
    sim.advance(5)  # let a fresh Get_Error reply replace the old one
    ok, msg = sim.core.request_manual_reset(sim.t, snap())
    assert not ok and 'Confirm' in msg
    assert sim.core.state == State.LOCKOUT
    ok, msg = sim.core.request_manual_reset(sim.t, snap())
    assert ok
    assert sim.core.inhibit
    sim.advance(10)
    assert sim.core.state == State.NORMAL and not sim.core.inhibit


def test_manual_reset_rejected_outside_lockout():
    sim = Sim()
    ok, _ = sim.core.request_manual_reset(sim.t, Snapshot(
        left=sim.ltel, right=sim.rtel))
    assert not ok




def test_startup_requires_voltage_from_both_controllers():
    sim = Sim(boot=False)

    # Right S1 continues sending heartbeats and answering Get_Error,
    # but does not answer voltage requests.
    sim.right.answers_voltage = False

    sim.advance(2.0)

    assert sim.core.inhibit
    assert sim.core.startup
    assert sim.core.state == State.NORMAL

    # Even with fresh voltage from the left controller,
    # startup must not release the motion inhibit.
    assert sim.ltel.v_time is not None
    assert sim.rtel.v_time is None

    sim.advance(5.0)

    assert sim.core.inhibit
    assert sim.core.state == State.LOCKOUT


def test_voltage_requests_sent_to_both_controllers():
    sim = Sim()
    voltage_frames = [
        f for f in sim.frames if f.label == 'get_voltage'
    ]

    assert voltage_frames
    assert any(
        f.node == 'left'
        and f.can_id == 0x7B7
        and f.rtr
        and not f.mutating
        for f in voltage_frames
    )
    assert any(
        f.node == 'right'
        and f.can_id == 0x7D7
        and f.rtr
        and not f.mutating
        for f in voltage_frames
    )


def test_voltage_telemetry_remains_fresh_during_normal_operation():
    sim = Sim()
    sim.advance(10.0)

    assert sim.core.state == State.NORMAL
    assert not sim.core.inhibit
    assert sim.ltel.v == pytest.approx(25.0)
    assert sim.rtel.v == pytest.approx(25.0)
    assert sim.t - sim.ltel.v_time < sim.cfg.telemetry_stale_s
    assert sim.t - sim.rtel.v_time < sim.cfg.telemetry_stale_s


# ---- protocol / mocked CAN ------------------------------------------------
def test_can_ids_match_obot2_table():
    assert oc.arb_id(61, oc.CMD_HEARTBEAT) == 0x7A1
    assert oc.arb_id(62, oc.CMD_HEARTBEAT) == 0x7C1
    assert oc.get_error_request(61) == (0x7A3, b'', True)
    assert oc.get_error_request(62) == (0x7C3, b'', True)
    assert oc.clear_errors(61) == (0x7B8, b'', False)
    assert oc.clear_errors(62) == (0x7D8, b'', False)
    assert oc.set_axis_state(61, 8) == (0x7A7, bytes.fromhex('08000000'),
                                         False)
    assert oc.set_axis_state(62, 8)[0] == 0x7C7
    # OBot-1 IDs stay different
    assert oc.get_error_request(0)[0] == 0x003
    assert oc.get_error_request(1)[0] == 0x023


def _frame(can_id, data, rtr=False):
    import struct
    flags = oc.CAN_RTR_FLAG if rtr else 0
    return struct.pack(oc.CAN_FRAME_FMT, can_id | flags, len(data),
                       data.ljust(8, b'\x00'))


def test_mocked_can_traffic_updates_telemetry():
    import struct
    left, right = make_nodes(61, 62)
    tel = CanTelemetry('can0', left, right)
    tel.ingest(_frame(0x7A1, struct.pack('<IBBB', 0x100, 1, 0, 0)), now=1.0)
    tel.ingest(_frame(0x7C1, struct.pack('<IBBB', 0, 8, 0, 0)), now=1.1)
    tel.ingest(_frame(0x7A3, struct.pack('<II', 0x100, 0)), now=1.2)
    tel.ingest(_frame(0x0B7, b'\x00' * 8), now=1.3)  # node 5: ignored
    tel.ingest(_frame(0x003, struct.pack('<II', 5, 0)), now=1.3)  # OBot-1 id
    tel.ingest(_frame(0x7B7, struct.pack('<ff', 26.5, 0.0)), now=1.4)
    tel.ingest(_frame(0x7A3, b'', rtr=True), now=1.5)  # our own RTR
    lt, rt, *_ = tel.snapshot_nodes()
    assert (lt.hb_error, lt.hb_state, lt.hb_time) == (0x100, 1, 1.0)
    assert (rt.hb_error, rt.hb_state) == (0, 8)
    assert (lt.ge_active, lt.ge_time) == (0x100, 1.2)
    assert lt.v == pytest.approx(26.5)
    assert rt.v is None  # missing voltage stays missing, never 0
    assert rt.ge_time is None


def test_nonfinite_voltage_is_ignored():
    import struct
    assert oc.parse_voltage(struct.pack('<f', math.nan)) is None
    assert oc.parse_voltage(b'\x00') is None
    assert oc.parse_heartbeat(b'\x00\x00') is None


def test_fault_log_record_contains_required_fields():
    sim = Sim()
    sim.fault_ov()
    sim.advance(DT * 2)
    rec = next(r for r in sim.logs if r['event'] == 'fault_detected')
    for key in ('robot', 'can_state', 'left', 'right', 'fault_class',
                'recovery_attempts_in_window', 'f1_release_confirmed',
                'lockout_reason', 't_monotonic'):
        assert key in rec
    assert rec['left']['active_error_names'] == ['DC_BUS_OVER_VOLTAGE']
    json.dumps(rec, default=str)
