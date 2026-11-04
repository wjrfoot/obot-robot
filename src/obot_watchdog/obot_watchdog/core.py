"""Pure, non-blocking OBot-2 fault state machine (no ROS, CAN or sleeps).

The ROS node feeds in a Snapshot every tick and executes the returned
actions. All times are monotonic seconds supplied by the caller, which keeps
the logic deterministic and testable.
"""

from dataclasses import dataclass, field
import enum
import math
from typing import List, Optional

from . import odrive_can as oc


class State(enum.Enum):
    NORMAL = 'NORMAL'
    FAULT_DETECTED = 'FAULT_DETECTED'
    WAIT_FOR_F1_RELEASE = 'WAIT_FOR_F1_RELEASE'
    RECOVERING = 'RECOVERING'
    VERIFYING = 'VERIFYING'
    LOCKOUT = 'LOCKOUT'


INFO = 'INFO'
WARNING = 'WARNING'
CRITICAL = 'CRITICAL'

V_NORMAL = 'NORMAL'
V_ELEVATED = 'ELEVATED'
V_UNSAFE = 'UNSAFE'
V_STALE = 'STALE'
V_UNVALIDATED = 'UNVALIDATED'

VERIFY_POLL_S = 0.5


@dataclass
class Config:
    robot_name: str = 'OBot-2'
    left_node_id: int = 61
    right_node_id: int = 62
    monitor_interval: float = 0.5
    telemetry_stale_s: float = 2.0
    joy_timeout_s: float = 1.0
    observation_window: float = 60.0
    max_recoveries: int = 3
    automatic_recovery_enabled: bool = False
    # Unset (NaN) values keep automatic recovery refused: not yet validated.
    verify_timeout_s: float = math.nan
    voltage_settle_timeout_s: float = math.nan
    voltage_min_v: float = math.nan
    voltage_normal_max_v: float = math.nan
    voltage_unsafe_v: float = math.nan
    recoverable_error_mask: int = oc.ERR_DC_BUS_OVER_VOLTAGE
    require_voltage_telemetry: bool = True
    command_gap_s: float = 0.5
    can_fault_confirmations: int = 2
    get_error_miss_limit: int = 2
    startup_grace_s: float = 5.0
    notification_min_interval_s: float = 10.0
    manual_reset_confirm_window_s: float = 30.0

    def voltage_configured(self):
        return not any(math.isnan(x) for x in (
            self.voltage_min_v, self.voltage_normal_max_v,
            self.voltage_unsafe_v))

    def recovery_criteria_configured(self):
        return (self.voltage_configured()
                and not math.isnan(self.verify_timeout_s)
                and not math.isnan(self.voltage_settle_timeout_s))


@dataclass
class NodeTel:
    name: str
    node_id: int
    hb_time: Optional[float] = None
    hb_error: int = 0
    hb_state: Optional[int] = None
    ge_time: Optional[float] = None
    ge_active: int = 0
    ge_disarm: int = 0
    v: Optional[float] = None
    v_time: Optional[float] = None


@dataclass
class Snapshot:
    left: NodeTel
    right: NodeTel
    can_state: Optional[str] = None
    can_time: Optional[float] = None
    can_seq: int = 0
    joy_time: Optional[float] = None
    f1_pressed: Optional[bool] = None
    cmd_path_safe: bool = False
    commanded_vel: Optional[tuple] = None
    measured_vel: Optional[tuple] = None

    def nodes(self):
        return (self.left, self.right)


@dataclass
class Notify:
    level: str
    text: str


@dataclass
class Frame:
    label: str
    node: str
    can_id: int
    data: bytes
    rtr: bool
    mutating: bool


@dataclass
class Log:
    record: dict


@dataclass
class Persist:
    fault_active: bool
    lockout_reason: Optional[str]


@dataclass
class Issue:
    kind: str
    node: Optional[str]
    eligible: bool
    text: str
    active: int = 0
    disarm: int = 0
    axis_state: Optional[int] = None


@dataclass
class Core:
    cfg: Config
    now0: float = 0.0
    state: State = State.NORMAL
    inhibit: bool = True
    startup: bool = True
    armed: bool = False
    lockout_reason: Optional[str] = None
    fault_time: float = 0.0
    fault_class: str = ''
    fault_text: str = ''
    fault_history: List[float] = field(default_factory=list)
    attempt_history: List[float] = field(default_factory=list)
    operator_authorized: bool = False
    f1_release_confirmed: bool = False
    actions: list = field(default_factory=list)

    def __post_init__(self):
        self._next_poll = 0.0
        self._poll_sent_at = None
        self._ge_miss = {}
        self._can_bad = 0
        self._can_seq = None
        self._last_notify = {}
        self._phase = ''
        self._phase_t = 0.0
        self._verify_t0 = 0.0
        self._verify_poll_t = 0.0
        self._precheck_t0 = 0.0
        self._reset_armed_until = None
        self._elevated_seen = False
        self._snap = None

    # ---- helpers -------------------------------------------------------
    def _notify(self, level, text, key=None):
        key = key or text
        last = self._last_notify.get(key)
        if (last is not None and level != CRITICAL
                and self._now - last < self.cfg.notification_min_interval_s):
            return
        self._last_notify[key] = self._now
        self.actions.append(Notify(level, f'{self.cfg.robot_name}: {text}'))

    def _persist(self):
        self.actions.append(Persist(
            self.state != State.NORMAL, self.lockout_reason))

    def _set_state(self, state, reason=''):
        if state != self.state:
            self._log('state_change', from_state=self.state.value,
                      to_state=state.value, reason=reason)
            self.state = state
            self._persist()

    def _diag(self):
        snap = self._snap
        out = {}
        if snap is not None:
            out['can_state'] = snap.can_state
            for tel in snap.nodes():
                out[tel.name] = {
                    'node_id': tel.node_id,
                    'heartbeat_error': tel.hb_error,
                    'axis_state': tel.hb_state,
                    'active_errors': tel.ge_active,
                    'disarm_reason': tel.ge_disarm,
                    'active_error_names': oc.decode_errors(
                        tel.hb_error | tel.ge_active | tel.ge_disarm),
                    'bus_voltage': tel.v,
                    'bus_voltage_level': self.voltage_level(tel),
                }
            out['commanded_velocity'] = snap.commanded_vel
            out['measured_velocity'] = snap.measured_vel
        return out

    def _log(self, event, **fields):
        record = {
            't_monotonic': self._now,
            'robot': self.cfg.robot_name,
            'event': event,
            'state': self.state.value,
            'fault_class': self.fault_class,
            'recovery_attempts_in_window': self._count(self.attempt_history),
            'faults_in_window': self._count(self.fault_history),
            'f1_release_confirmed': self.f1_release_confirmed,
            'lockout_reason': self.lockout_reason,
        }
        record.update(self._diag())
        record.update(fields)
        self.actions.append(Log(record))

    def _count(self, history):
        return sum(1 for t in history
                   if self._now - t <= self.cfg.observation_window)

    def _fresh(self, stamp):
        return (stamp is not None
                and self._now - stamp <= self.cfg.telemetry_stale_s)

    def voltage_level(self, tel):
        if tel.v is None or not self._fresh(tel.v_time):
            return V_STALE
        cfg = self.cfg
        if not cfg.voltage_configured():
            return V_UNVALIDATED
        if tel.v < cfg.voltage_min_v or tel.v >= cfg.voltage_unsafe_v:
            return V_UNSAFE
        if tel.v > cfg.voltage_normal_max_v:
            return V_ELEVATED
        return V_NORMAL

    def _f1_released_fresh(self, snap):
        return (snap.joy_time is not None
                and snap.f1_pressed is False
                and self._now - snap.joy_time <= self.cfg.joy_timeout_s)

    def _name_of(self, tel):
        return 'Left' if tel is self._snap.left else 'Right'

    # ---- assessment ----------------------------------------------------
    def _assess(self, snap):
        """Return the list of current Issues from fresh data only."""
        cfg, now = self.cfg, self._now
        issues = []
        in_grace = now - self.now0 < cfg.startup_grace_s and self.startup

        for tel in snap.nodes():
            who = self._name_of(tel)
            hb_fresh = self._fresh(tel.hb_time)
            if not hb_fresh:
                if not in_grace:
                    issues.append(Issue(
                        'HEARTBEAT_STALE', who, False,
                        f'{who} S1 heartbeat missing or stale'))
                continue
            ge_fresh = (tel.ge_time is not None and
                        now - tel.ge_time
                        <= cfg.monitor_interval + cfg.telemetry_stale_s)
            active = tel.ge_active if ge_fresh else 0
            disarm = tel.ge_disarm if ge_fresh else 0
            mask = tel.hb_error | active | disarm
            if mask:
                eligible = (mask & ~cfg.recoverable_error_mask) == 0
                names = ','.join(oc.decode_errors(mask))
                issues.append(Issue(
                    'S1_ERROR', who, eligible,
                    f'{who} S1 motor fault ({names})',
                    active, disarm, tel.hb_state))
            elif self.armed and tel.hb_state != oc.AXIS_STATE_CLOSED_LOOP:
                issues.append(Issue(
                    'NOT_CLOSED_LOOP', who, True,
                    f'{who} S1 left closed-loop control '
                    f'(state {tel.hb_state})',
                    axis_state=tel.hb_state))
            if self._ge_miss.get(tel.name, 0) >= cfg.get_error_miss_limit:
                issues.append(Issue(
                    'GET_ERROR_NO_RESPONSE', who, False,
                    f'{who} S1 not answering Get_Error'))
            level = self.voltage_level(tel)
            if (level == V_STALE and cfg.require_voltage_telemetry
                    and not in_grace):
                issues.append(Issue(
                    'VOLTAGE_STALE', who, False,
                    f'{who} S1 bus-voltage telemetry missing or stale'))
            elif level == V_UNSAFE:
                issues.append(Issue(
                    'VOLTAGE_UNSAFE', who, True,
                    f'{who} S1 bus voltage unsafe ({tel.v:.1f} V)'))

        if snap.can_time is not None and snap.can_seq != self._can_seq:
            self._can_seq = snap.can_seq
            if snap.can_state == 'ERROR-ACTIVE':
                self._can_bad = 0
            else:
                self._can_bad += 1
        can_age_bad = (snap.can_time is None
                       or now - snap.can_time > 3 * cfg.monitor_interval)
        if can_age_bad and not in_grace:
            issues.append(Issue('CAN_UNKNOWN', None, False,
                                'CAN interface health unknown'))
        elif self._can_bad >= cfg.can_fault_confirmations:
            issues.append(Issue(
                'CAN_STATE', None, False,
                f'CAN interface fault ({snap.can_state})'))
        return issues

    @staticmethod
    def _classify(issues):
        if any(i.kind in ('HEARTBEAT_STALE', 'GET_ERROR_NO_RESPONSE',
                          'VOLTAGE_STALE', 'CAN_UNKNOWN', 'CAN_STATE')
               for i in issues):
            return 'COMMUNICATION_LOSS', 'TELEMETRY_OR_COMMUNICATION_LOSS'
        if any(not i.eligible for i in issues):
            return 'NONRECOVERABLE', 'NONRECOVERABLE_ERROR'
        return 'TRANSIENT_ELIGIBLE', ''

    # ---- polling -------------------------------------------------------
    def _poll(self, snap):
        if self._now < self._next_poll:
            return
        for tel in snap.nodes():
            if self._poll_sent_at is not None:
                got = tel.ge_time is not None and tel.ge_time >= self._poll_sent_at
                self._ge_miss[tel.name] = (
                    0 if got else self._ge_miss.get(tel.name, 0) + 1)
        self._poll_sent_at = self._now
        self._next_poll = self._now + self.cfg.monitor_interval
        for tel in snap.nodes():
            can_id, data, rtr = oc.get_error_request(tel.node_id)
            self.actions.append(
                Frame('get_error', tel.name, can_id, data, rtr, False))

            can_id, data, rtr = oc.get_bus_voltage_request(tel.node_id)
            self.actions.append(
                Frame('get_voltage', tel.name, can_id, data, rtr, False))

    # ---- main entry ----------------------------------------------------
    def tick(self, now, snap):
        self._now = now
        self._snap = snap
        self.actions = []
        self._poll(snap)
        handler = {
            State.NORMAL: self._normal,
            State.WAIT_FOR_F1_RELEASE: self._wait,
            State.RECOVERING: self._recovering,
            State.VERIFYING: self._verifying,
            State.LOCKOUT: self._lockout,
        }.get(self.state)
        if handler:
            handler(snap)
        return self.actions

    def restore(self, now, fault_active, lockout_reason):
        """Fail closed after a watchdog restart that interrupted a fault."""
        self._now = now
        if fault_active or lockout_reason:
            self.actions = []
            self.startup = False
            self.inhibit = True
            self.lockout_reason = (lockout_reason
                                   or 'WATCHDOG_RESTARTED_DURING_FAULT')
            self.fault_class = 'RESTORED'
            self.state = State.LOCKOUT
            self._notify(
                CRITICAL,
                'Repeated or persistent motor fault. Automatic recovery '
                'disabled. Manual investigation required. '
                f'(restored lockout: {self.lockout_reason})',
                key='lockout')
            return self.actions
        return []

    # ---- NORMAL --------------------------------------------------------
    def _normal(self, snap):
        issues = self._assess(snap)
        if issues:
            self._detect(issues)
            return

        levels = [self.voltage_level(t) for t in snap.nodes()]
        if V_ELEVATED in levels:
            if not self._elevated_seen:
                self._elevated_seen = True
                self._notify(WARNING, 'Elevated DC bus voltage observed.',
                             key='elevated')
        else:
            self._elevated_seen = False

        if all(self._fresh(t.hb_time)
               and t.hb_state == oc.AXIS_STATE_CLOSED_LOOP
               and t.hb_error == 0 for t in snap.nodes()):
            self.armed = True

        if self.inhibit and self.startup:
            ready = (
                self.armed
                and all(lv in (V_NORMAL, V_ELEVATED, V_UNVALIDATED)
                        for lv in levels)
                and self._f1_released_fresh(snap)
                and snap.cmd_path_safe)
            if ready:
                self.startup = False
                self.inhibit = False
                self._log('startup_release')
                self._persist()

    # ---- FAULT_DETECTED ------------------------------------------------
    def _detect(self, issues):
        self.inhibit = True
        self.fault_time = self._now
        self.f1_release_confirmed = False
        self.operator_authorized = False
        self.startup = False
        self.fault_history.append(self._now)
        cls, reason = self._classify(issues)
        self.fault_class = cls
        self.fault_text = '; '.join(i.text for i in issues)
        self._set_state(State.FAULT_DETECTED, self.fault_text)
        self._notify(
            WARNING,
            f'{self.fault_text}. Motion inhibited. '
            'Waiting for F1 release.', key='fault')
        self._log('fault_detected', issues=[
            {'kind': i.kind, 'node': i.node, 'eligible': i.eligible,
             'active_errors': i.active, 'disarm_reason': i.disarm,
             'axis_state': i.axis_state} for i in issues])

        if cls != 'TRANSIENT_ELIGIBLE':
            self._lockout_enter(reason)
        elif self._count(self.fault_history) > self.cfg.max_recoveries:
            self.fault_class = 'REPEATED'
            self._lockout_enter('REPEATED_FAULTS_WITHIN_WINDOW')
        elif not self.cfg.automatic_recovery_enabled:
            self._lockout_enter('AUTOMATIC_RECOVERY_DISABLED')
        else:
            self._set_state(State.WAIT_FOR_F1_RELEASE, 'transient eligible')

    # ---- WAIT_FOR_F1_RELEASE -------------------------------------------
    def _wait(self, snap):
        self.inhibit = True
        if (snap.joy_time is not None and snap.joy_time > self.fault_time
                and self._f1_released_fresh(snap)):
            self.f1_release_confirmed = True
            self._log('f1_release_confirmed')
            self._phase = 'PRECHECK'
            self._precheck_t0 = self._now
            self._set_state(State.RECOVERING, 'F1 released')

    # ---- RECOVERING ----------------------------------------------------
    def _recovering(self, snap):
        self.inhibit = True
        cfg = self.cfg
        if not self._f1_released_fresh(snap):
            self.f1_release_confirmed = False
            self._set_state(State.WAIT_FOR_F1_RELEASE,
                            'F1 pressed or joystick silent')
            self.fault_time = self._now
            return

        if self._phase == 'CLEAR_SENT':
            if self._now - self._phase_t >= cfg.command_gap_s:
                for tel in snap.nodes():
                    self._frame('set_closed_loop', tel,
                                oc.set_axis_state(
                                    tel.node_id, oc.AXIS_STATE_CLOSED_LOOP))
                self._verify_t0 = self._now
                self._verify_poll_t = self._now
                self._set_state(State.VERIFYING, 'recovery commands sent')
            return

        authorised = (cfg.automatic_recovery_enabled
                      or self.operator_authorized)
        if not authorised:
            self._lockout_enter('AUTOMATIC_RECOVERY_DISABLED')
            return
        if not cfg.recovery_criteria_configured():
            self._lockout_enter('RECOVERY_CRITERIA_NOT_CONFIGURED')
            return
        if self._count(self.attempt_history) >= cfg.max_recoveries:
            self.fault_class = 'REPEATED'
            self._lockout_enter('RECOVERY_LIMIT_REACHED')
            return

        issues = self._assess(snap)
        bad = [i for i in issues if not i.eligible]
        if bad:
            self._lockout_enter(
                'DISQUALIFYING_FAULT: ' + '; '.join(i.text for i in bad))
            return

        levels = [self.voltage_level(t) for t in snap.nodes()]
        if any(lv != V_NORMAL for lv in levels):
            if self._now - self._precheck_t0 > cfg.voltage_settle_timeout_s:
                stale = V_STALE in levels
                if not stale:
                    self.fault_class = 'PERSISTENT_VOLTAGE'
                self._lockout_enter(
                    'VOLTAGE_TELEMETRY_UNAVAILABLE' if stale
                    else 'PERSISTENT_UNSAFE_BUS_VOLTAGE')
            return

        self.attempt_history.append(self._now)
        self._log('recovery_attempt',
                  attempt=self._count(self.attempt_history))
        for tel in snap.nodes():
            self._frame('clear_errors', tel, oc.clear_errors(tel.node_id))
        self._phase = 'CLEAR_SENT'
        self._phase_t = self._now

    def _frame(self, label, tel, spec):
        can_id, data, rtr = spec
        self.actions.append(Frame(label, tel.name, can_id, data, rtr, True))

    # ---- VERIFYING -----------------------------------------------------
    def _verifying(self, snap):
        self.inhibit = True
        cfg = self.cfg
        if not self._f1_released_fresh(snap):
            self.f1_release_confirmed = False
            self.fault_time = self._now
            self._set_state(State.WAIT_FOR_F1_RELEASE,
                            'F1 pressed or joystick silent during verify')
            return

        if self._now - self._verify_poll_t >= VERIFY_POLL_S:
            self._verify_poll_t = self._now
            for tel in snap.nodes():
                can_id, data, rtr = oc.get_error_request(tel.node_id)
                self.actions.append(
                    Frame('get_error', tel.name, can_id, data, rtr, False))

        t0 = self._verify_t0
        ok = snap.cmd_path_safe and snap.can_state == 'ERROR-ACTIVE'
        for tel in snap.nodes():
            ok = (ok
                  and tel.hb_time is not None and tel.hb_time > t0
                  and tel.hb_error == 0
                  and tel.hb_state == oc.AXIS_STATE_CLOSED_LOOP
                  and tel.ge_time is not None and tel.ge_time > t0
                  and tel.ge_active == 0 and tel.ge_disarm == 0
                  and self._fresh(tel.hb_time)
                  and self.voltage_level(tel) == V_NORMAL)
        if ok:
            self.inhibit = False
            self.armed = True
            self.fault_class = ''
            self._log('recovery_result', result='SUCCESS')
            self._notify(
                INFO,
                'Both motor controllers recovered. Closed-loop control '
                'verified. Ready for operation.', key='recovered')
            self._set_state(State.NORMAL, 'recovery verified')
            return

        if self._now - t0 > cfg.verify_timeout_s:
            self._log('recovery_result', result='FAILED')
            self._notify(WARNING,
                         'Recovery unsuccessful. Motion remains inhibited.',
                         key='recovery_failed')
            if self._count(self.attempt_history) >= cfg.max_recoveries:
                self.fault_class = 'REPEATED'
                self._lockout_enter('RECOVERY_FAILED_AT_LIMIT')
            else:
                self._phase = 'PRECHECK'
                self._precheck_t0 = self._now
                self._set_state(State.RECOVERING, 'retry after failed verify')

    # ---- LOCKOUT -------------------------------------------------------
    def _lockout_enter(self, reason):
        self.inhibit = True
        self.lockout_reason = reason
        self._log('lockout', reason=reason)
        self._notify(
            CRITICAL,
            'Repeated or persistent motor fault. Automatic recovery '
            f'disabled. Manual investigation required. ({reason})',
            key='lockout')
        self._set_state(State.LOCKOUT, reason)
        self._persist()

    def _lockout(self, snap):
        self.inhibit = True

    def request_manual_reset(self, now, snap):
        """Two-step, revalidating reset. Returns (accepted, message)."""
        self._now = now
        self._snap = snap
        self.actions = []
        cfg = self.cfg
        if self.state != State.LOCKOUT:
            return False, f'Not in LOCKOUT (state {self.state.value}).'
        problems = []
        if not self._f1_released_fresh(snap):
            problems.append('F1 not confirmed released by fresh /joy data')
        issues = self._assess(snap)
        hard = [i for i in issues if not i.eligible]
        problems += [i.text for i in hard]
        if cfg.voltage_configured():
            for tel in snap.nodes():
                if self.voltage_level(tel) != V_NORMAL:
                    problems.append(f'{tel.name} bus voltage not NORMAL')
        elif any(self.voltage_level(t) == V_STALE for t in snap.nodes()):
            problems.append('bus-voltage telemetry stale')
        if problems:
            self._reset_armed_until = None
            self._log('manual_reset_rejected', problems=problems)
            return False, 'Revalidation failed: ' + '; '.join(problems)

        if (self._reset_armed_until is None
                or now > self._reset_armed_until):
            self._reset_armed_until = now + cfg.manual_reset_confirm_window_s
            return False, (
                'Revalidation passed. Confirm by calling the reset service '
                f'again within {cfg.manual_reset_confirm_window_s:.0f} s '
                'after you have checked the underlying fault.')

        self._reset_armed_until = None
        self._log('manual_reset_accepted', previous_reason=self.lockout_reason)
        self.fault_history.clear()
        self.attempt_history.clear()
        self.lockout_reason = None
        self.fault_class = ''
        if issues:
            self.operator_authorized = True
            self.fault_time = now
            self.f1_release_confirmed = False
            self._set_state(State.WAIT_FOR_F1_RELEASE, 'operator reset')
        else:
            self.startup = True
            self.now0 = now
            self._set_state(State.NORMAL, 'operator reset, healthy')
        self._persist()
        return True, 'Lockout cleared. Motion stays inhibited until verified.'
