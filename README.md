# Reach Alpha 5 ROS 2 Driver :mechanical_arm:

The Alpha 5 driver is a collection of ROS 2 packages designed
to support ROS 2 integration with the [Reach Alpha 5 manipulator](https://reachrobotics.com/products/manipulators/reach-alpha/).

## Main features

The main features of the Alpha 5 driver are:

- Integration of the [Reach System Serial Protocol](https://reach-robotics.github.io/reach_robotics_sdk/documentation/index.html#)
  for hardware communication
- `ros2_control` integration for manipulator position, velocity, and joint trajectory control
- Visualization support using RViz2
- Integration with MoveIt2 for motion planning
- Gazebo support for kinematic simulation

## Installation

The Alpha 5 driver requires at least ROS 2 Humble and can be installed from
source on a Linux machine. To install the Alpha 5 driver from source, clone
this project to the `src` directory of your ROS 2 workspace:

```bash
git clone git@github.com:evan-palmer/alpha.git
```

After cloning the project, install the ROS 2 dependencies using rosdep,
replacing `<ROS_DISTRO>` with the desired ROS 2 distro:

```bash
rosdep update && \
rosdep install -y --from-paths src --ignore-src --rosdistro <ROS_DISTRO>
```

## Quick start

A ROS 2 launch file has been provided to start the Alpha 5 driver. To launch the
driver using default arguments, run

```bash
ros2 launch alpha_bringup alpha.launch.py
```

A full description of the launch arguments and their respective default values
can be obtained by running the following:

```bash
ros2 launch alpha_bringup alpha.launch.py --show-args
```

## Getting help

If you have questions regarding usage of the Alpha 5 driver or regarding
contributing to this project, please ask a question on our
[Dicussions](https://github.com/evan-palmer/alpha/discussions) board!

## License

The Alpha 5 ROS 2 driver is released under the MIT license.
