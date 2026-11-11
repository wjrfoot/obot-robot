"""Pure watchdog state-machine tests; no ROS or CAN hardware is required."""

from obot_watchdog import odrive_can as oc
from obot_watchdog.core import (
    Config, Core, Frame, Log, NodeTel, Notify, Snapshot, State,
)


class SimulatedController:
    def __init__(self, name, node_id):
        self.name = name
        self.node_id = node_id
        self.errors = 0
        self.disarm = 0
        self.state = oc.AXIS_STATE_CLOSED_LOOP
        self.voltage = 24.0
        self.clear_allowed = True
        self.closed_loop_allowed = True
        self.get_error_enabled = True
        self.clear_count = 0
        self.closed_loop_count = 0
        self.telemetry = NodeTel(name, node_id)


class WatchdogRig:
    def __init__(self, **config):
        self.now = 0.0
        self.can_state = 'ERROR-ACTIVE'
        self.left = SimulatedController('left', 61)
        self.right = SimulatedController('right', 62)
        self.core = Core(Config(**config), now0=self.now)
        self.actions = []
        self.tick()

    def tick(self, dt=0.1):
        self.now += dt
        for controller in (self.left, self.right):
            tel = controller.telemetry
            tel.hb_time = self.now
            tel.hb_error = controller.errors
            tel.hb_state = controller.state
            tel.ge_active = controller.errors
            tel.ge_disarm = controller.disarm
            tel.v = controller.voltage
            tel.v_time = self.now

        actions = self.core.tick(self.now, Snapshot(
            self.left.telemetry, self.right.telemetry,
            can_state=self.can_state, can_time=self.now))
        self.actions.extend(actions)
        for action in actions:
            if not isinstance(action, Frame):
                continue
            controller = getattr(self, action.node)
            if action.label == 'get_error':
                if controller.get_error_enabled:
                    controller.telemetry.ge_time = self.now
            elif action.label == 'clear_errors':
                controller.clear_count += 1
                if controller.clear_allowed:
                    controller.errors = 0
                    controller.disarm = 0
                    controller.state = 1
            elif action.label == 'set_closed_loop':
                controller.closed_loop_count += 1
                if (controller.closed_loop_allowed
                        and controller.errors == 0):
                    controller.state = oc.AXIS_STATE_CLOSED_LOOP
        return actions

    def run_for(self, seconds):
        steps = int(seconds / 0.1)
        for _ in range(steps):
            self.tick()


def mutating(actions):
    return [action for action in actions
            if isinstance(action, Frame) and action.mutating]


def test_healthy_controllers_only_receive_immediate_diagnostics():
    rig = WatchdogRig()
    frames = [action for action in rig.actions if isinstance(action, Frame)]
    assert {frame.label for frame in frames} == {'get_error', 'get_voltage'}
    assert all(not frame.mutating for frame in frames)
    assert rig.core.state is State.NORMAL


def test_fault_log_includes_controller_diagnostics():
    rig = WatchdogRig()
    rig.left.errors = oc.ERR_DC_BUS_OVER_VOLTAGE
    actions = rig.tick()
    record = next(action.record for action in actions
                  if isinstance(action, Log)
                  and action.record['event'] == 'fault_detected')
    assert record['controller'] == 'left'
    assert record['node_id'] == 61
    assert record['heartbeat_error'] == oc.ERR_DC_BUS_OVER_VOLTAGE
    assert record['bus_voltage'] == 24.0
    assert record['t_monotonic'] == rig.now
    assert any(isinstance(action, Notify) for action in actions)


def test_fault_recovery_clears_and_verifies_only_affected_controller():
    rig = WatchdogRig()
    rig.left.errors = oc.ERR_DC_BUS_OVER_VOLTAGE
    rig.left.disarm = 7
    rig.left.state = 1
    rig.tick()
    rig.run_for(4.0)

    assert rig.left.clear_count == 1
    assert rig.left.closed_loop_count == 1
    assert rig.left.state == oc.AXIS_STATE_CLOSED_LOOP
    assert rig.right.clear_count == 0
    assert rig.right.closed_loop_count == 0
    assert rig.core.state is State.NORMAL
    assert any(
        isinstance(action, Notify) and 'recovered' in action.text
        for action in rig.actions)


def test_disarm_reason_without_error_bits_still_attempts_clear():
    rig = WatchdogRig()
    rig.right.disarm = 7
    actions = rig.tick()
    assert any(action.label == 'clear_errors'
               for action in mutating(actions))
    record = next(action.record for action in actions
                  if isinstance(action, Log)
                  and action.record['event'] == 'fault_detected')
    assert record['fault_class'] == 'S1_DISARM'
    assert record['disarm_reason'] == 7


def test_left_and_right_faults_recover_independently():
    rig = WatchdogRig()
    rig.left.errors = oc.ERR_DC_BUS_OVER_VOLTAGE
    rig.left.state = 1
    rig.right.errors = oc.ERR_DC_BUS_OVER_VOLTAGE
    rig.right.state = 1
    rig.tick()
    rig.run_for(4.0)

    assert rig.left.clear_count == rig.right.clear_count == 1
    assert rig.left.state == rig.right.state == oc.AXIS_STATE_CLOSED_LOOP
    results = [action.record for action in rig.actions
               if isinstance(action, Log)
               and action.record['event'] == 'recovery_result']
    assert {record['controller'] for record in results
            if record.get('result') == 'SUCCESS'} == {'left', 'right'}


def test_nonrecoverable_error_is_reported_without_reset_commands():
    rig = WatchdogRig()
    rig.right.errors = 0x400
    rig.right.state = 1
    actions = rig.tick()
    assert not mutating(actions)
    assert rig.core._controllers['right'].phase is State.PERSISTENT
    assert any(
        isinstance(action, Notify)
        and action.level == 'CRITICAL'
        for action in actions)


def test_can_link_fault_is_reported_and_blocks_recovery_commands():
    rig = WatchdogRig()
    rig.left.errors = oc.ERR_DC_BUS_OVER_VOLTAGE
    rig.left.state = 1
    rig.can_state = 'BUS-OFF'
    actions = rig.tick()
    assert not mutating(actions)
    assert any(
        isinstance(action, Log)
        and action.record['event'] == 'communication_fault'
        and action.record['reason'] == 'CAN link is BUS-OFF'
        for action in actions)

    rig.can_state = 'ERROR-ACTIVE'
    actions = rig.tick()
    assert any(action.label == 'clear_errors'
               for action in mutating(actions))


def test_persistent_recoverable_fault_has_bounded_retries():
    rig = WatchdogRig(max_recoveries=3, retry_interval_s=1.0,
                      verify_timeout_s=0.5)
    rig.left.errors = oc.ERR_DC_BUS_OVER_VOLTAGE
    rig.left.state = 1
    rig.left.clear_allowed = False
    rig.tick()
    rig.run_for(12.0)

    assert rig.left.clear_count == 3
    assert rig.core._controllers['left'].phase is State.PERSISTENT
    count_at_lockout = rig.left.clear_count
    rig.run_for(10.0)
    assert rig.left.clear_count == count_at_lockout
    assert any(
        isinstance(action, Log)
        and action.record.get('result') == 'PERSISTENT_FAULT'
        for action in rig.actions)


def test_recovery_requires_post_command_heartbeat_and_get_error():
    rig = WatchdogRig()
    rig.left.errors = oc.ERR_DC_BUS_OVER_VOLTAGE
    rig.left.state = 1
    rig.tick()

    for _ in range(8):
        actions = rig.tick()
        if any(isinstance(action, Frame)
               and action.label == 'set_closed_loop'
               for action in actions):
            break
    status = rig.core._controllers['left']
    assert status.phase is State.VERIFYING

    rig.left.state = oc.AXIS_STATE_CLOSED_LOOP
    rig.left.errors = 0
    rig.left.telemetry.hb_time = status.verify_after
    rig.left.telemetry.ge_time = status.verify_after
    rig.tick(0.1)
    assert status.phase is State.VERIFYING

    rig.run_for(1.0)
    assert rig.core._controllers['left'].phase is State.NORMAL


def test_repeated_missing_get_error_responses_log_communication_fault():
    rig = WatchdogRig(get_error_miss_limit=2)
    rig.left.get_error_enabled = False
    rig.right.get_error_enabled = False
    rig.run_for(4.0)
    records = [action.record for action in rig.actions
               if isinstance(action, Log)
               and action.record['event'] == 'communication_fault']
    assert any(record['controller'] == 'left'
               and record['reason'] == 'Get_Error response missing'
               for record in records)


def test_odrive_error_protocol_helpers():
    assert oc.get_error_request(61) == (0x7A3, b'', True)
    assert oc.clear_errors(62) == (0x7D8, b'', False)
    assert oc.set_axis_state(61, 8) == (0x7A7, b'\x08\x00\x00\x00', False)
    assert oc.decode_errors(oc.ERR_DC_BUS_OVER_VOLTAGE) == [
        'DC_BUS_OVER_VOLTAGE']
