#!/usr/bin/env bash
set -e

source /opt/ros/jazzy/setup.bash
source "$HOME/obot_ws/install/setup.bash"

export ROS_DOMAIN_ID=5
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$HOME/obot_ws/src/obot_bringup/config/cyclonedds_wifi.xml"

exec ros2 launch obot_bringup obot_control.launch.py
