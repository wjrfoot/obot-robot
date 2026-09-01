# OBot Description

ROS 2 Jazzy URDF/Xacro package for the ODrive-based SprayBot robot (OBot).

## Install dependencies

```bash
sudo apt update
sudo apt install -y ros-jazzy-xacro ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher-gui ros-jazzy-rviz2
```

## Build

Place this package in `~/obot_ws/src/obot_description`, then:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/obot_ws
colcon build --symlink-install --packages-select obot_description
source ~/obot_ws/install/setup.bash
```

## Validate the Xacro

```bash
xacro ~/obot_ws/src/obot_description/urdf/obot.urdf.xacro > /tmp/obot.urdf
check_urdf /tmp/obot.urdf
```

If `check_urdf` is not installed:

```bash
sudo apt install -y liburdfdom-tools
```

## Display in RViz

```bash
ros2 launch obot_description view_obot.launch.py
```

The same package can be installed on both the OBot and the Ubuntu laptop.
For routine visualization, RViz is best run on the laptop while the robot
publishes its TF/joint states over ROS_DOMAIN_ID=5.

## Known placeholder

The rear caster wheel width is temporarily modeled as 1.0 in because its
measured width has not yet been supplied.
