# OBot Robot

ROS 2 software and configuration that runs on the OBot mobile robot (`pi-bot-1`).

OBot is the current mobile-robot platform for the SprayBot research project. It uses ODrive S1 motor controllers with differential drive and runs its robot-side ROS 2 stack on a Raspberry Pi 5.

## Platform

- Ubuntu 24.04
- ROS 2 Jazzy
- `ROS_DOMAIN_ID=5`
- CycloneDDS
- Raspberry Pi 5
- ODrive S1 motor controllers
- CAN bus at 1 Mbit/s
- RPLIDAR
- Differential-drive `ros2_control`

## Repository Layout

- `src/obot` — robot application nodes and launch files
- `src/obot_bringup` — robot bringup and configuration
- `src/obot_description` — URDF/Xacro, controllers, and RViz configuration
- `src/obot_msgs` — OBot-specific ROS messages
- `src/obot_valve_interfaces` — spray valve action and completion message
- `src/obot_valve_controller` — spray valve action server (robot) and PS4 client (laptop); see its README
- `bin` — robot diagnostic and recovery utilities
- `service_scripts` — scripts used by systemd
- `systemd` — systemd service definitions
- `patches` — local fixes for external dependencies

Generated colcon directories (`build`, `install`, and `log`) are not stored in Git.

## External Dependencies

Two upstream ROS repositories are installed separately under `src/` and are intentionally excluded from this repository.

### ros_odrive

Upstream repository: https://github.com/odriverobotics/ros_odrive

The OBot installation contains a local ROS 2 Jazzy build fix adding `find_package(pluginlib REQUIRED)` to `odrive_ros2_control/CMakeLists.txt`.

The modification is preserved in `patches/ros_odrive_jazzy.patch`.

### sllidar_ros2

Upstream repository: https://github.com/Slamtec/sllidar_ros2

The OBot installation corrects the A2M12 launch filename supplied by the upstream repository.

The modification is preserved in `patches/sllidar_a2m12_launch.patch`.

## CycloneDDS

The active robot DDS configuration is machine-specific and is excluded from Git:

`src/obot_bringup/config/cyclonedds_wifi.xml`

A portable template is provided:

`src/obot_bringup/config/cyclonedds_wifi.example.xml`

Copy the example to `cyclonedds_wifi.xml` and set the network-interface name for the robot's ROS Wi-Fi interface.

## Build

    cd ~/obot_ws
    source /opt/ros/jazzy/setup.bash
    colcon build --symlink-install
    source install/setup.bash

## Automatic Bringup

The robot uses `systemd/obot.service`, which invokes `service_scripts/start_obot.sh`.

The bringup launches:

    ros2 launch obot_bringup obot_control.launch.py

## Git Workflow

Before making changes:

    cd ~/obot_ws
    git pull
    git status

After testing a change:

    git status
    git add <files>
    git commit -m "Describe the change"
    git push
