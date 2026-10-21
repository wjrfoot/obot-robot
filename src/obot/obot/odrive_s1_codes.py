"""ODrive S1 CANSimple definitions, verified against the ODrive documentation.

Sources (checked 2026-10-21):
  https://docs.odriverobotics.com/v/latest/manual/can-protocol.html
  https://docs.odriverobotics.com/v/latest/fibre_types/com_odriverobotics_ODrive.html

Receive-only: nothing in this module builds or transmits a CAN frame.
"""
import struct

# CANSimple 11-bit arbitration id = (node_id << 5) | cmd_id.
CMD_HEARTBEAT = 0x01
CMD_GET_ERROR = 0x03
CMD_GET_TEMPERATURE = 0x15
CMD_GET_BUS_VOLTAGE_CURRENT = 0x17

MONITORED_CMDS = (
    CMD_HEARTBEAT,
    CMD_GET_ERROR,
    CMD_GET_TEMPERATURE,
    CMD_GET_BUS_VOLTAGE_CURRENT,
)

# Heartbeat is cyclic by default (100 ms). Get_Error, Get_Temperature and
# Get_Bus_Voltage_Current are DISABLED by default (error_msg_rate_ms,
# temperature_msg_rate_ms, bus_voltage_msg_rate_ms = 0) and only appear on the
# bus if configured, or if some other host requests them with an RTR frame.
DEFAULT_CYCLIC = {
    CMD_HEARTBEAT: True,
    CMD_GET_ERROR: False,
    CMD_GET_TEMPERATURE: False,
    CMD_GET_BUS_VOLTAGE_CURRENT: False,
}

CAN_RTR_FLAG = 0x40000000
CAN_EFF_FLAG = 0x80000000
CAN_ERR_FLAG = 0x20000000

# Value used in published uint32 fields when a code is not available.
CODE_UNAVAILABLE = 0xFFFFFFFF

# ODrive.Error bit flags. Used for active_errors and disarm_reason. The
# heartbeat Axis_Error field is (active_errors | disarm_reason).
ERROR_FLAGS = {
    0x00000001: ('INITIALIZING',
                 'Initializing or reconfiguring; normal shortly after power-up'),
    0x00000002: ('SYSTEM_LEVEL',
                 'Unexpected system error (memory corruption, stack overflow, '
                 'frozen thread, assert); indicates a firmware bug'),
    0x00000004: ('TIMING_ERROR',
                 'Internal hard timing requirement violated (overloaded)'),
    0x00000008: ('MISSING_ESTIMATE',
                 'Position, velocity or phase estimate needed but invalid'),
    0x00000010: ('BAD_CONFIG', 'ODrive configuration invalid or incomplete'),
    0x00000020: ('DRV_FAULT', 'Gate driver chip reported an error'),
    0x00000040: ('MISSING_INPUT',
                 'No value provided for input_pos, input_vel or input_torque'),
    0x00000100: ('DC_BUS_OVER_VOLTAGE',
                 'DC bus voltage exceeded dc_bus_overvoltage_trip_level'),
    0x00000200: ('DC_BUS_UNDER_VOLTAGE',
                 'DC bus voltage fell below dc_bus_undervoltage_trip_level'),
    0x00000400: ('DC_BUS_OVER_CURRENT', 'Too much DC current was drawn'),
    0x00000800: ('DC_BUS_OVER_REGEN_CURRENT',
                 'Too much DC current was regenerated'),
    0x00001000: ('CURRENT_LIMIT_VIOLATION',
                 'Motor current exceeded current_hard_max'),
    0x00002000: ('MOTOR_OVER_TEMP',
                 'Motor thermistor above temp_limit_upper'),
    0x00004000: ('INVERTER_OVER_TEMP',
                 'FET/inverter thermistor above temp_limit_upper'),
    0x00008000: ('VELOCITY_LIMIT_VIOLATION',
                 'Estimated velocity exceeded vel_limit_tolerance * vel_limit'),
    0x00010000: ('POSITION_LIMIT_VIOLATION', 'Position limit violated'),
    0x00020000: ('REQUESTED_CURRENT_TOO_HIGH',
                 'Requested calibration current exceeded effective current limit'),
    0x01000000: ('WATCHDOG_TIMER_EXPIRED', 'Axis watchdog timer expired'),
    0x02000000: ('ESTOP_REQUESTED',
                 'Estop requested externally (CAN estop message or endstop)'),
    0x04000000: ('SPINOUT_DETECTED',
                 'Electrical vs mechanical power mismatch (spinout)'),
    0x08000000: ('BRAKE_RESISTOR_DISARMED',
                 'Another component failed; commonly the brake resistor disarmed'),
    0x10000000: ('THERMISTOR_DISCONNECTED',
                 'Motor thermistor enabled but disconnected'),
    0x40000000: ('CALIBRATION_ERROR',
                 'A calibration procedure failed; see procedure_result'),
}

# Bits that are not faults on their own.
BENIGN_ERROR_MASK = 0x00000001  # INITIALIZING

AXIS_STATES = {
    0: 'UNDEFINED', 1: 'IDLE', 2: 'STARTUP_SEQUENCE',
    3: 'FULL_CALIBRATION_SEQUENCE', 4: 'MOTOR_CALIBRATION',
    6: 'ENCODER_INDEX_SEARCH', 7: 'ENCODER_OFFSET_CALIBRATION',
    8: 'CLOSED_LOOP_CONTROL', 9: 'LOCKIN_SPIN', 10: 'ENCODER_DIR_FIND',
    11: 'HOMING', 12: 'ENCODER_HALL_POLARITY_CALIBRATION',
    13: 'ENCODER_HALL_PHASE_CALIBRATION', 14: 'ANTICOGGING_CALIBRATION',
    15: 'HARMONIC_CALIBRATION', 16: 'HARMONIC_CALIBRATION_COMMUTATION',
}

PROCEDURE_RESULTS = {
    0: 'SUCCESS', 1: 'BUSY', 2: 'CANCELLED', 3: 'DISARMED', 4: 'NO_RESPONSE',
    5: 'POLE_PAIR_CPR_MISMATCH', 6: 'PHASE_RESISTANCE_OUT_OF_RANGE',
    7: 'PHASE_INDUCTANCE_OUT_OF_RANGE', 8: 'UNBALANCED_PHASES',
    9: 'INVALID_MOTOR_TYPE', 10: 'ILLEGAL_HALL_STATE', 11: 'TIMEOUT',
    12: 'HOMING_WITHOUT_ENDSTOP', 13: 'INVALID_STATE', 14: 'NOT_CALIBRATED',
    15: 'NOT_CONVERGING', 16: 'REQUESTED_CURRENT_TOO_HIGH',
}

AXIS_STATE_CLOSED_LOOP = 8


def decode_error_mask(mask):
    """Return [{'bit', 'name', 'description'}] for every set bit, low to high.

    Bits that are not defined in the documentation are kept, named
    UNKNOWN_0x<bit>, so no numeric information is lost.
    """
    if mask is None:
        return []
    out = []
    for i in range(32):
        bit = 1 << i
        if mask & bit:
            name, desc = ERROR_FLAGS.get(
                bit, ('UNKNOWN_0x%08X' % bit, 'Not defined in the ODrive docs'))
            out.append({'bit': bit, 'name': name, 'description': desc})
    return out


def describe_error_mask(mask):
    """Short text such as 'DC_BUS_OVER_VOLTAGE|ESTOP_REQUESTED' ('NONE' if 0)."""
    if mask is None:
        return 'UNAVAILABLE'
    if mask == 0:
        return 'NONE'
    return '|'.join(e['name'] for e in decode_error_mask(mask))


def axis_state_name(value):
    return AXIS_STATES.get(value, 'UNKNOWN_%d' % value)


def procedure_result_name(value):
    return PROCEDURE_RESULTS.get(value, 'UNKNOWN_%d' % value)


def arbitration_id(node_id, cmd_id):
    return (node_id << 5) | cmd_id


def split_arbitration_id(can_id):
    """Return (node_id, cmd_id) from an 11-bit standard CAN id."""
    return (can_id >> 5) & 0x3F, can_id & 0x1F


def decode_heartbeat(data):
    """Heartbeat 0x01: u32 Axis_Error, u8 Axis_State, u8 Procedure_Result, u8 Traj_Done."""
    if len(data) < 7:
        return None
    axis_error, state, result, traj = struct.unpack_from('<IBBB', data)
    return {'axis_error': axis_error, 'axis_state': state,
            'procedure_result': result, 'trajectory_done': bool(traj)}


def decode_get_error(data):
    """Get_Error 0x03: u32 Active_Errors, u32 Disarm_Reason."""
    if len(data) < 8:
        return None
    active, disarm = struct.unpack_from('<II', data)
    return {'active_errors': active, 'disarm_reason': disarm}


def decode_get_temperature(data):
    """Get_Temperature 0x15: f32 FET_Temperature, f32 Motor_Temperature (deg C)."""
    if len(data) < 8:
        return None
    fet, motor = struct.unpack_from('<ff', data)
    return {'fet_temp_c': fet, 'motor_temp_c': motor}


def decode_get_bus_voltage_current(data):
    """Get_Bus_Voltage_Current 0x17: f32 Bus_Voltage (V), f32 Bus_Current (A)."""
    if len(data) < 8:
        return None
    volts, amps = struct.unpack_from('<ff', data)
    return {'bus_voltage_v': volts, 'bus_current_a': amps}


DECODERS = {
    CMD_HEARTBEAT: decode_heartbeat,
    CMD_GET_ERROR: decode_get_error,
    CMD_GET_TEMPERATURE: decode_get_temperature,
    CMD_GET_BUS_VOLTAGE_CURRENT: decode_get_bus_voltage_current,
}
