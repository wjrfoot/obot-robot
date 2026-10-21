# Page 1 summary for the CYD, published by obot_status_node at 1 Hz on /obot/status.
# Fixed-size fields only (no strings / unbounded sequences) so micro-ROS can use it.
# Every item is a state code below. A value is never OK unless fresh data proves it:
# missing or expired telemetry is STALE, not-yet-seen/undeterminable is UNKNOWN.

uint8 UNKNOWN=0
uint8 OK=1
uint8 WARN=2
uint8 FAULT=3
uint8 STALE=4

# Nav2 lifecycle (ACTIVE=OK, INACTIVE=WARN) and control mode
uint8 CONTROL_UNKNOWN=0
uint8 CONTROL_MANUAL=1
uint8 CONTROL_NAV2=2

# Valve feedback from the separate valve ESP32 (/spray/valve_state)
uint8 VALVE_UNKNOWN=0
uint8 VALVE_OFF=1
uint8 VALVE_ON=2
uint8 VALVE_STALE=3

# Pi clock at publish time. The CYD has no reliable clock; it shows age since receipt.
builtin_interfaces/Time stamp

uint8 telemetry_link      # status aggregator itself
uint8 nav2
uint8 localization        # AMCL or SLAM, whichever is running
uint8 lidar               # fresh /scan
uint8 camera1             # fresh front camera image
uint8 camera2             # fresh rear camera image
uint8 can_interface       # can0 link state
uint8 wifi_link
uint8 control_mode        # CONTROL_*
uint8 valve               # VALVE_*

# Index 0 = left (CAN node_id 0), index 1 = right (CAN node_id 1)
uint8[2] motor_heartbeat  # state code
uint8[2] motor_axis_state # raw ODrive axis state (0 = unknown/undefined)
uint8[2] motor_error      # state code: OK if no active error, FAULT if any

uint8 localization_mode   # 0 unknown, 1 amcl, 2 slam
uint16 fault_count        # persistent total, from the Pi fault log
bool alert                # true if any item is FAULT
