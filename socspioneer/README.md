# The SOCSPIONEER Package

`socspioneer` is a package of configurations and helper nodes for use
with the School of Computer Science (University of Birmingham),
Intelligent Robotics module.

## Installation

**NOTE**: *This part assumes basic understanding of Linux terminal and
commandline usage. Basic understanding of ROS workflow and package
organisation is also important (eg. [Building a catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)).*

### Ideal Working Environment

- Ubuntu 20.04
- [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)
(desktop-full)

**NOTE**: *Installing ROS Noetic by default installs Python 3 packages. For compatibility and lack of conflicts, stick to
using Python 3 when writing Python nodes.*

### Install dependencies

- `sudo apt install ros-$ROS_DISTRO-pr2-teleop ros-$ROS_DISTRO-joy ros-$ROS_DISTRO-slam-gmapping ros-$ROS_DISTRO-map-server`.
- *Not needed when using simulation; Only required for real Pioneer
3DX robot.* `sudo apt install ros-$ROS_DISTRO-p2os-driver ros-$ROS_DISTRO-p2os-launch ros-$ROS_DISTRO-p2os-urdf ros-$ROS_DISTRO-p2os-teleop`.

### Build package

- Clone this repo to the `src` directory of your catkin workspace.
- Build the catkin workspace (`catkin_make` or `catkin build`).

**NOTE: The catkin workspace should be sourced each time a new
terminal session is loaded (run `source devel/setup.bash`). Alternatively,
add the line `source <catkin_ws>/devel/setup.bash` to your `.bashrc`
file to avoid repeating it every time.**

## Testing Simulation and Installation

If everything installed correctly, the following steps should provide
a very simplistic simulation of a robot in a provided world map.

1. In one terminal, run roscore.
2. In another, run `rosrun stage_ros stageros <catkin_ws>/src/socspioneer/data/meeting.world`.
This should start a simple simulated world with a robot and a map.
3. In a third terminal, run `roslaunch socspioneer keyboard_teleop.launch`.

This would allow you to move the robot using keyboard commands. Note that
when controlling using the keyboard control, the terminal where the
keyboard control node is running should be in focus (click on the terminal
before using the keys to control the robot).

## Simulator Usage

**NOTE**: *This part assumes basic understanding of ROS, ROS topics,
messages, nodes, etc.*

Running `rosrun stage_ros stageros <.world file>` will start the
simulator with a robot and an obstacle the provided world. The
robot and object can be interacted with using the mouse or using
ROS topics, nodes, etc. The world view can also be changed using
the mouse.

- Pressing `R` on keyboard toggles between 2D and 3D views. 
- `D` key toggles laser field of view visualisation.

The simulator publishes the following (important) topics. By
subscribing to these topics, you can access different sensor
information from the robot.

| ROS Topic | Data | ROS Message Type |
| ------ | ------ | ------ |
| `/odom` | The odometry information from the robot wheel encoders. | [`nav_msgs/Odometry`](http://docs.ros.org/kinetic/api/nav_msgs/html/msg/Odometry.html) |
| `/base_scan` | Laser scan data from the laser scanner at the front of the robot. | [`sensor_msgs/LaserScan`](http://docs.ros.org/kinetic/api/sensor_msgs/html/msg/LaserScan.html) |

The simulator subscribes to the following topics. You can control
the robot using this.

| ROS Topic | Data | ROS Message Type |
| ------ | ------ | ------ |
| `/cmd_vel` | Velocity commands to the robot's wheel motors. | [`geometry_msgs/Twist`](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Twist.html) |
