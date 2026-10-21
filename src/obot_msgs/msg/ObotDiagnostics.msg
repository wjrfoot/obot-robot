# Page 2 data for the CYD, published by obot_status_node at 1 Hz on /obot/diagnostics.
# Fixed-size fields only so micro-ROS can use it. Unavailable floats are NaN, never 0.
# Index 0 = left (CAN node_id 0), index 1 = right (CAN node_id 1).
#
# Voltage min/max are *observed* extrema of the sampled CAN data. They are not true
# instantaneous peaks: short transients between samples can be missed.

builtin_interfaces/Time stamp

# Raspberry Pi
float32 pi_cpu_temp_c          # NaN if unavailable
float32 pi_cpu_percent         # NaN if not collected
bool throttle_available        # false if vcgencmd is missing/failed
uint32 throttle_flags          # raw `vcgencmd get_throttled`
# Decoded for convenience (bits 0-3 current, 16-19 historical)
bool undervoltage_now
bool throttled_now
bool undervoltage_since_boot
bool throttled_since_boot
# No numeric Pi 5V value: it is not available from vcgencmd.

# ODrive S1 controllers
float32[2] bus_voltage_v       # current
float32[2] bus_voltage_session_min_v
float32[2] bus_voltage_session_max_v
float32[2] bus_voltage_alltime_min_v
float32[2] bus_voltage_alltime_max_v
float32[2] controller_temp_c   # NaN if not reported
# Raw ODrive S1 ODrive.Error bit masks (decoded by the Pi and in the fault log).
# active_errors: Get_Error Active_Errors if fresh, else heartbeat Axis_Error (which is
#   active|disarm combined). disarm_reason: Get_Error Disarm_Reason, or 0xFFFFFFFF when
#   not observed (Get_Error is not cyclic by default; the node never requests it).
uint32[2] active_errors
uint32[2] disarm_reason
uint16[2] fault_count          # persistent, per controller
builtin_interfaces/Time[2] last_fault_stamp   # sec = 0 if none

# CAN interface
uint8 can_state                # 0 unknown, 1 error-active, 2 error-warning, 3 error-passive, 4 bus-off, 5 stopped
uint32 can_rx_errors           # 0 if not available
uint32 can_tx_errors           # 0 if not available

builtin_interfaces/Time session_extrema_reset_stamp
