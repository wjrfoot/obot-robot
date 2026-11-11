"""Pure per-controller ODrive fault detection and recovery logic."""

from dataclasses import dataclass, field
import enum
from typing import Optional

from . import odrive_can as oc


class State(enum.Enum):
    NORMAL = 'NORMAL'
    RECOVERING = 'RECOVERING'
    VERIFYING = 'VERIFYING'
    RETRY_WAIT = 'RETRY_WAIT'
    PERSISTENT = 'PERSISTENT'


INFO = 'INFO'
WARNING = 'WARNING'
CRITICAL = 'CRITICAL'


@dataclass
class Config:
    robot_name: str = 'OBot-2'
    left_node_id: int = 61
    right_node_id: int = 62
    monitor_interval: float = 1.0
    telemetry_stale_s: float = 2.0
    get_error_miss_limit: int = 5
    recoverable_error_mask: int = oc.ERR_DC_BUS_OVER_VOLTAGE
    max_recoveries: int = 3
    retry_interval_s: float = 5.0
    verify_timeout_s: float = 3.0
    command_gap_s: float = 0.5


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
class _ControllerRecovery:
    phase: State = State.NORMAL
    attempts: int = 0
    phase_time: float = 0.0
    verify_after: float = 0.0
    signature: Optional[tuple] = None
    reported_issue: Optional[str] = None


@dataclass
class Core:
    """Track and recover each S1 independently, without operator inputs."""

    cfg: Config
    now0: float = 0.0
    actions: list = field(default_factory=list)

    def __post_init__(self):
        self._now = self.now0
        self._next_poll = self.now0
        self._last_get_error_sent = None
        self._get_error_misses = {}
        self._snap = None
        self._controllers = {
            'left': _ControllerRecovery(),
            'right': _ControllerRecovery(),
        }

    @property
    def state(self):
        phases = {status.phase for status in self._controllers.values()}
        for phase in (
                State.PERSISTENT, State.VERIFYING, State.RECOVERING,
                State.RETRY_WAIT):
            if phase in phases:
                return phase
        return State.NORMAL

    def _fresh(self, stamp):
        return (stamp is not None
                and 0.0 <= self._now - stamp <= self.cfg.telemetry_stale_s)

    def _diagnostic(self, tel):
        ge_fresh = self._fresh(tel.ge_time)
        heartbeat_error = tel.hb_error if self._fresh(tel.hb_time) else 0
        active = tel.ge_active if ge_fresh else 0
        disarm = tel.ge_disarm if ge_fresh else 0
        errors = heartbeat_error | active
        return {
            'controller': tel.name,
            'node_id': tel.node_id,
            'heartbeat_error': heartbeat_error,
            'active_errors': active,
            'disarm_reason': disarm,
            'axis_state': tel.hb_state if self._fresh(tel.hb_time) else None,
            'error_names': oc.decode_errors(errors),
            'bus_voltage': tel.v if self._fresh(tel.v_time) else None,
            'can_state': self._snap.can_state,
            'errors': errors,
        }

    def _record(self, event, tel, **fields):
        record = {
            't_monotonic': self._now,
            'robot': self.cfg.robot_name,
            'event': event,
        }
        record.update(self._diagnostic(tel))
        record.update(fields)
        self.actions.append(Log(record))

    def _notify(self, level, tel, message):
        self.actions.append(Notify(
            level, f'{self.cfg.robot_name}: {tel.name} S1 {message} '
            f'(node {tel.node_id})'))

    def _frame(self, label, tel, spec):
        can_id, data, rtr = spec
        self.actions.append(
            Frame(label, tel.name, can_id, data, rtr, True))

    def _poll(self, snap):
        if self._now < self._next_poll:
            return
        if self._last_get_error_sent is not None:
            for tel in snap.nodes():
                received = (
                    tel.ge_time is not None
                    and tel.ge_time >= self._last_get_error_sent)
                self._get_error_misses[tel.name] = (
                    0 if received else
                    self._get_error_misses.get(tel.name, 0) + 1)

        self._last_get_error_sent = self._now
        self._next_poll = self._now + self.cfg.monitor_interval
        for tel in snap.nodes():
            can_id, data, rtr = oc.get_error_request(tel.node_id)
            self.actions.append(
                Frame('get_error', tel.name, can_id, data, rtr, False))
            can_id, data, rtr = oc.get_bus_voltage_request(tel.node_id)
            self.actions.append(
                Frame('get_voltage', tel.name, can_id, data, rtr, False))

    def tick(self, now, snap):
        """Return diagnostic, recovery, logging and notification actions."""
        self._now = now
        self._snap = snap
        self.actions = []
        self._poll(snap)
        for tel in snap.nodes():
            self._tick_controller(tel)
        return self.actions

    def _tick_controller(self, tel):
        status = self._controllers[tel.name]
        diag = self._diagnostic(tel)
        fresh = self._fresh(tel.hb_time)
        errors = diag['errors']
        closed_loop = (
            fresh and tel.hb_state == oc.AXIS_STATE_CLOSED_LOOP
            and errors == 0 and diag['disarm_reason'] == 0)
        signature = (errors, diag['disarm_reason'], diag['axis_state'])
        can_state = self._snap.can_state
        if can_state and can_state != 'ERROR-ACTIVE':
            issue = f'CAN link is {can_state}'
            if status.reported_issue != issue:
                self._record('communication_fault', tel, reason=issue)
                self._notify(WARNING, tel, issue)
            status.reported_issue = issue
            if (status.phase == State.VERIFYING
                    and self._now - status.phase_time
                    >= self.cfg.verify_timeout_s):
                self._retry_or_persist(tel, status, issue)
            return
        if (status.reported_issue
                and status.reported_issue.startswith('CAN link is ')):
            status.reported_issue = None
        if not fresh:
            issue = 'heartbeat missing or stale'
            if status.reported_issue != issue:
                self._record('communication_fault', tel, reason=issue)
                self._notify(WARNING, tel, issue)
                status.reported_issue = issue
            if (status.phase == State.VERIFYING
                    and self._now - status.phase_time
                    >= self.cfg.verify_timeout_s):
                self._retry_or_persist(tel, status, issue)
            return
        if status.reported_issue == 'heartbeat missing or stale':
            status.reported_issue = None

        if closed_loop and status.phase in (
                State.PERSISTENT, State.RETRY_WAIT):
            self._record(
                'recovery_result', tel, result='SUCCESS',
                attempts=status.attempts,
                detail='controller returned to closed-loop without retry')
            self._notify(INFO, tel, 'recovered; closed-loop verified')
            self._controllers[tel.name] = _ControllerRecovery()
            return

        if closed_loop and status.phase == State.NORMAL:
            if (self._get_error_misses.get(tel.name, 0)
                    >= self.cfg.get_error_miss_limit):
                issue = 'Get_Error response missing'
                if status.reported_issue != issue:
                    self._record('communication_fault', tel, reason=issue)
                    self._notify(WARNING, tel, issue)
                status.reported_issue = issue
            else:
                status.reported_issue = None
            return

        if closed_loop and status.phase == State.RECOVERING:
            return

        if not closed_loop and status.phase == State.PERSISTENT:
            if signature != status.signature:
                self._begin_fault(tel, status, errors, signature)
            return

        if not closed_loop and status.phase == State.RETRY_WAIT:
            if signature != status.signature:
                status.signature = signature
                status.phase_time = self._now
            if (self._now - status.phase_time
                    >= self.cfg.retry_interval_s):
                self._begin_fault(tel, status, errors, signature, retry=True)
            return

        if status.phase == State.NORMAL:
            self._begin_fault(tel, status, errors, signature)
            return

        if status.phase == State.RECOVERING:
            if self._now - status.phase_time < self.cfg.command_gap_s:
                return
            self._request_closed_loop(tel, status)
            return

        if status.phase == State.VERIFYING:
            ge_fresh_after_command = (
                tel.ge_time is not None
                and tel.ge_time > status.verify_after
                and self._fresh(tel.ge_time))
            heartbeat_fresh_after_command = (
                tel.hb_time is not None
                and tel.hb_time > status.verify_after
                and self._fresh(tel.hb_time))
            if heartbeat_fresh_after_command and ge_fresh_after_command:
                if (tel.hb_error == 0
                        and tel.hb_state == oc.AXIS_STATE_CLOSED_LOOP
                        and tel.ge_active == 0 and tel.ge_disarm == 0):
                    self._record(
                        'recovery_result', tel, result='SUCCESS',
                        attempts=status.attempts)
                    self._notify(INFO, tel, 'recovered; closed-loop verified')
                    self._controllers[tel.name] = _ControllerRecovery()
                    return
            if self._now - status.phase_time >= self.cfg.verify_timeout_s:
                self._retry_or_persist(tel, status, 'verification timed out')
            return


    def _begin_fault(self, tel, status, errors, signature, retry=False):
        if not retry:
            status.signature = signature
            status.attempts = 0
            if errors:
                fault_class = 'S1_ERROR'
            elif signature[1]:
                fault_class = 'S1_DISARM'
            else:
                fault_class = 'NOT_CLOSED_LOOP'
            self._record(
                'fault_detected', tel,
                fault_class=fault_class)
            self._notify(WARNING, tel, 'fault detected')

        unrecoverable = errors & ~self.cfg.recoverable_error_mask
        if unrecoverable:
            status.phase = State.PERSISTENT
            status.signature = signature
            self._record(
                'recovery_result', tel, result='NOT_RECOVERABLE',
                attempts=status.attempts)
            self._notify(
                CRITICAL, tel, 'fault is not eligible for automatic recovery')
            return

        if errors and not self.cfg.recoverable_error_mask:
            status.phase = State.PERSISTENT
            status.signature = signature
            self._record(
                'recovery_result', tel, result='NOT_RECOVERABLE',
                attempts=status.attempts)
            self._notify(CRITICAL, tel, 'no recoverable errors configured')
            return

        if status.attempts >= self.cfg.max_recoveries:
            status.phase = State.PERSISTENT
            status.signature = signature
            self._record(
                'recovery_result', tel, result='PERSISTENT_FAULT',
                attempts=status.attempts)
            self._notify(
                CRITICAL, tel,
                'fault persists after bounded recovery attempts')
            return

        status.attempts += 1
        status.phase_time = self._now
        status.signature = signature
        status.phase = State.RECOVERING
        self._record(
            'recovery_attempt', tel, attempt=status.attempts,
            retry=retry)
        self._frame('clear_errors', tel, oc.clear_errors(tel.node_id))

    def _request_closed_loop(self, tel, status):
        self._frame(
            'set_closed_loop', tel,
            oc.set_axis_state(tel.node_id, oc.AXIS_STATE_CLOSED_LOOP))
        can_id, data, rtr = oc.get_error_request(tel.node_id)
        self.actions.append(
            Frame('get_error', tel.name, can_id, data, rtr, False))
        status.phase = State.VERIFYING
        status.phase_time = self._now
        status.verify_after = self._now

    def _retry_or_persist(self, tel, status, reason):
        self._record(
            'recovery_result', tel, result='FAILED',
            attempts=status.attempts, reason=reason)
        self._notify(
            WARNING, tel,
            f'recovery attempt {status.attempts} unsuccessful')
        if status.attempts >= self.cfg.max_recoveries:
            status.phase = State.PERSISTENT
            self._record(
                'recovery_result', tel, result='PERSISTENT_FAULT',
                attempts=status.attempts)
            self._notify(
                CRITICAL, tel,
                'fault persists after bounded recovery attempts')
            return
        status.phase = State.RETRY_WAIT
        status.phase_time = self._now
