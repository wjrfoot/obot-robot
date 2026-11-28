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

The OBot installation also sets `Input_Mode = INPUT_MODE_VEL_RAMP` in velocity control mode, preserved in `patches/ros_odrive_vel_ramp.patch`. The installation is based on upstream commit `efe8f77`.

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

## Deferred Investigation: OBot-2 Startup Failure

On 2026-11-23, an OBot-2 startup failure showed the following:

- The CAN interface was ERROR-PASSIVE with a TX error counter of 128 and no
  successful packet exchanges. Its parent device was `spi1.0`.
- Both wheel positions were NaN, causing automatic deactivation of the
  differential drive controller.
- A reboot followed by a complete power cycle restored operation. After
  recovery, the CAN parent device was `spi0.0`.
- Both S1 controllers returned to closed-loop control without active errors.
  Both ROS controllers were active, and wheel position and velocity data were
  valid.

The SPI parent-device difference is an observation, not an established root
cause. Investigation is deferred.

## Laptop Motion Test

### Viewer motion request clock issue

On 2026-11-27, OBot-2's viewer TURN and directional requests were rejected
with `Expired or out-of-order viewer request`. The old viewer used its own
ROS wall clock to set a deadline 0.25 seconds ahead; the robot compared that
deadline with its independently maintained clock and rejected values over one
second into the future. A read-only STATUS diagnostic with a fresh viewer
identity reproduced the failure while the robot clock was approximately
1.095 seconds behind the laptop. The combined message did not distinguish
this condition from expiration, non-finite deadlines or sequence errors.

The replacement protocol issues one-use movement permits on `/click_drive`
with expiry measured using the robot's monotonic clock. Increasing viewer
sequences reject duplicates and reordering; STOP and manual release invalidate
outstanding permits without being rejected for their own sequence. Before its
first movement, a new viewer waits for an acknowledged STOP barrier. A failed
network can still prevent a STOP request from reaching the robot; the
directional lease remains an independent robot-side timeout. This is not a hardware emergency stop.
Both robot and laptop must build and run the matching updated `obot_msgs`
interface. See the laptop README for its clean-build procedure.

`obot_cmd_motion` is a terminal diagnostic for comparing commanded dead-reckoning
with wheel odometry. Run it on `ros-24` after sourcing the ROS 2 Jazzy and OBot
workspace setup files and joining the same ROS domain/DDS network as `pi-bot-2`:

    cd ~/obot_ws
    source /opt/ros/jazzy/setup.bash
    source install/setup.bash
    ros2 launch obot obot_cmd_motion.launch.py

The launch starts the command node and RViz. The terminal accepts `f N`/`b N`
for feet, `F N`/`B N` for meters, `r N`/`l N` for degrees, `s` to stop,
`c`/`z` to clear both paths and reset the origin, `p` to print the comparison,
`?` for help, and `q` to stop and quit. Ctrl-C also publishes zero commands
before exit. Optional `linear_speed` and `angular_speed` launch arguments set
the constant test speeds; the node does not add software acceleration ramps.

On `pi-bot-2`, the running bringup must use
`twist_mux_obot2.yaml`, which includes `/cmd_vel_test` at priority 90. The
OBot 2 bringup launch accepts it with
`twist_mux_config:=twist_mux_obot2.yaml`. The
watchdog topic and lock retain higher priority, and joystick teleoperation can
override motion tests. The laptop node observes
`/diff_drive_controller/cmd_vel` without tracking command sequence numbers.
Occasional missing or reordered messages do not abort the test; stale odometry
or mux feedback does stop it and discard queued commands. On reaching the
requested target it publishes zero, waits for observed zero output, and then
waits for odometry to settle. This does not bypass or replace the watchdog; a
higher-priority source may prevent the test node's zero from reaching the
controller.

The node publishes `/cmd_path`, `/actual_path`, and `/cmd_odom`; it only
subscribes to `/diff_drive_controller/odom` and does not modify actual
odometry. The two paths share the current odometry pose and heading whenever
the test origin is established. RViz uses `odom` as its fixed frame and
displays the robot description, both paths and commanded odometry. It requires
the robot's `/robot_description` and valid TF; NaN wheel joint positions can
prevent the affected wheel transforms from being displayed.

Commanded odometry integrates previously
observed mux velocities over consecutive local callback execution times,
measured with a monotonic clock;
it does not count the time before the first observed output. These local
callback times are not the middleware arrival, robot forwarding or
controller-receipt times. Header stamps do not provide the integration time.
Network delay, executor stalls, batched
callbacks, unobserved short overrides, and message loss can make
the estimated trajectory inaccurate; short overrides may be impossible to
detect from this topic alone. The node integrates output from any mux source
and cannot identify its publisher or verify that an observed zero came from
the test command; avoid competing commands during a test. Reaching the target
and forwarding zero also take time, so
motion can overshoot. Use this as a diagnostic in an appropriate test area,
not as a safety controller or closed-loop distance controller.
For completed tests, logs report the progress at target detection, local time
to observed zero output, and extra commanded progress. Rotation logs also compare
the latest received odometry yaw at target detection, zero output and settling.
These snapshots and their reported ages help distinguish command-side delay
from subsequent measured rotation; they are not synchronized controller-side
measurements, and do not by themselves identify its physical cause.
Target-crossing logs include the previous progress, callback interval and
callback-to-zero-publication duration. A per-command summary reports maximum
timer, send, and mux callback gaps and sampled path publication duration with
history point counts. RViz path messages are published at approximately 2 Hz,
with at most 500 evenly sampled points each, including the first and latest
points. Full-resolution history is retained until `c` or `z` resets it; memory
usage therefore grows with test duration. The former 10,000-point `path_limit`
cap has been removed. Path publication still runs on the motion executor and
can delay callbacks; bounded sampling reduces work but is not a real-time
guarantee.

## Gazebo Simulation

Install the ROS 2 Jazzy Gazebo Sim dependencies:

    sudo apt install ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge

Build and launch the robot in the included empty world:

    cd ~/Workspaces/obot-robot
    source /opt/ros/jazzy/setup.bash
    colcon build --symlink-install --packages-select obot_description
    source install/setup.bash
    ros2 launch obot_description sim.launch.py

Drive it by publishing `geometry_msgs/msg/Twist` on `/model/obot/cmd_vel`.
The simulated LiDAR publishes `sensor_msgs/msg/LaserScan` on `/scan`.

The maps in `~/maps` can be converted into static Gazebo walls with:

        python3 src/obot_description/tools/map_to_gazebo.py \
            ~/maps/map.yaml src/obot_description/worlds/map.sdf

The repository includes generated worlds for `map.yaml`, `map2.yaml`, and
`map3.yaml`. Select one at launch with `map:=map2.sdf` or `map:=map3.sdf`.

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
