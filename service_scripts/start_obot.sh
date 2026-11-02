#!/usr/bin/env bash
set -e

source /opt/ros/jazzy/setup.bash
source "$HOME/obot_ws/install/setup.bash"

export ROS_DOMAIN_ID=5
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$HOME/obot_ws/src/obot_bringup/config/cyclonedds_wifi.xml"

# Optional per-robot twist_mux file (OBot-2: twist_mux_obot2.yaml).
# Unset keeps the original twist_mux.yaml.
exec ros2 launch obot_bringup obot_control.launch.py \
  ${OBOT_TWIST_MUX_CONFIG:+twist_mux_config:=$OBOT_TWIST_MUX_CONFIG}
