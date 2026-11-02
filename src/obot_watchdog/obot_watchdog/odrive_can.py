"""ODrive CANSimple helpers (pure functions, no I/O).

Arbitration ID = (node_id << 5) | cmd_id (11-bit standard frames).
Message layouts follow the ODrive CAN protocol documentation:
  Heartbeat 0x01 (S1 -> host): Axis_Error u32, Axis_State u8, ...
  Get_Error 0x03 (RTR request): Active_Errors u32, Disarm_Reason u32
  Set_Axis_State 0x07: Axis_Requested_State u32
  Get_Bus_Voltage_Current 0x17: Bus_Voltage f32, Bus_Current f32
  Clear_Errors 0x18: no payload sent (optional Identify byte omitted)
"""

import math
import struct

CMD_HEARTBEAT = 0x01
CMD_GET_ERROR = 0x03
CMD_SET_AXIS_STATE = 0x07
CMD_GET_BUS_VOLTAGE_CURRENT = 0x17
CMD_CLEAR_ERRORS = 0x18

AXIS_STATE_CLOSED_LOOP = 8

CAN_FRAME_FMT = '=IB3x8s'
CAN_FRAME_SIZE = struct.calcsize(CAN_FRAME_FMT)
CAN_EFF_FLAG = 0x80000000
CAN_RTR_FLAG = 0x40000000
CAN_ERR_FLAG = 0x20000000
CAN_SFF_MASK = 0x7FF

ERR_DC_BUS_OVER_VOLTAGE = 0x00000100

# ODrive.Error bit masks (S1 firmware 0.6.x); verify against installed firmware.
ODRIVE_ERRORS = {
    0x00000001: 'INITIALIZING',
    0x00000002: 'SYSTEM_LEVEL',
    0x00000004: 'TIMING_ERROR',
    0x00000008: 'MISSING_ESTIMATE',
    0x00000010: 'BAD_CONFIG',
    0x00000020: 'DRV_FAULT',
    0x00000040: 'MISSING_INPUT',
    0x00000100: 'DC_BUS_OVER_VOLTAGE',
    0x00000200: 'DC_BUS_UNDER_VOLTAGE',
    0x00000400: 'DC_BUS_OVER_CURRENT',
    0x00000800: 'DC_BUS_OVER_REGEN_CURRENT',
    0x00001000: 'CURRENT_LIMIT_VIOLATION',
    0x00002000: 'MOTOR_OVER_TEMP',
    0x00004000: 'INVERTER_OVER_TEMP',
    0x00008000: 'VELOCITY_LIMIT_VIOLATION',
    0x00010000: 'POSITION_LIMIT_VIOLATION',
    0x00020000: 'REQUESTED_CURRENT_TOO_HIGH',
    0x01000000: 'WATCHDOG_TIMER_EXPIRED',
    0x02000000: 'ESTOP_REQUESTED',
    0x04000000: 'SPINOUT_DETECTED',
    0x08000000: 'BRAKE_RESISTOR_DISARMED',
    0x10000000: 'THERMISTOR_DISCONNECTED',
    0x40000000: 'CALIBRATION_ERROR',
}


def decode_errors(mask):
    names = [name for bit, name in ODRIVE_ERRORS.items() if mask & bit]
    unknown = mask & ~sum(ODRIVE_ERRORS)
    if unknown:
        names.append(f'UNKNOWN_0x{unknown:08X}')
    return names


def arb_id(node_id, cmd):
    if not 0 <= node_id <= 0x3F:
        raise ValueError(f'CAN node id out of range: {node_id}')
    return (node_id << 5) | cmd


def get_error_request(node_id):
    return arb_id(node_id, CMD_GET_ERROR), b'', True


def clear_errors(node_id):
    return arb_id(node_id, CMD_CLEAR_ERRORS), b'', False


def set_axis_state(node_id, state):
    return (arb_id(node_id, CMD_SET_AXIS_STATE),
            struct.pack('<I', state), False)


def parse_frame(raw):
    """Return (node_id, cmd, data) for a standard data frame, else None."""
    if len(raw) < CAN_FRAME_SIZE:
        return None
    can_id, dlc, data = struct.unpack(CAN_FRAME_FMT, raw[:CAN_FRAME_SIZE])
    if can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG):
        return None
    can_id &= CAN_SFF_MASK
    return can_id >> 5, can_id & 0x1F, data[:min(dlc, 8)]


def parse_heartbeat(data):
    """Return (axis_error, axis_state) or None."""
    if len(data) < 5:
        return None
    return struct.unpack('<IB', data[:5])


def parse_get_error(data):
    """Return (active_errors, disarm_reason) or None."""
    if len(data) < 8:
        return None
    return struct.unpack('<II', data[:8])


def parse_voltage(data):
    """Return bus voltage in volts, or None if absent or non-finite."""
    if len(data) < 4:
        return None
    value = struct.unpack('<f', data[:4])[0]
    return value if math.isfinite(value) else None
