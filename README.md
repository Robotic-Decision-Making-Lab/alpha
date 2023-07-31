# Reach Alpha 5 ROS 2 Driver :mechanical_arm:

The Alpha 5 driver is a collection of ROS 2 packages designed
to support ROS 2 integration with the [Reach Alpha 5 manipulator](https://reachrobotics.com/products/manipulators/reach-alpha/).

## :warning: Disclaimer :warning:

This is an independent project, and is not affiliated with or maintained by
Reach Robotics. Please refer to the [Reach Robotics SDK](https://github.com/Reach-Robotics/reach_robotics_sdk/tree/master)
for all official software.

## Main features

The main features of the Alpha 5 driver are:

- Integration of the [Reach System Serial Protocol](https://reach-robotics.github.io/reach_robotics_sdk/documentation/index.html#)
  for hardware communication
- ros2_control integration for manipulator position, velocity, and joint
  trajectory control
- Gazebo Garden integration for simulation
- Visualization support using RViz2
- Integration with MoveIt2 for motion planning

## Installation

The Alpha 5 driver is currently supported on Linux, and is available for the ROS
distributions Humble and Rolling. To install the Alpha 5 driver, first clone
this project to the `src` directory of your ROS workspace, replacing
`$ROS_DISTRO` with the desired ROS distribution or `main` for Rolling:

```bash
git clone -b $ROS_DISTRO git@github.com:evan-palmer/alpha.git
```

After cloning the project, install the ROS dependencies using `rosdep`, again,
replacing `$ROS_DISTRO` with the desired ROS distribution:

```bash
rosdep update && \
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
```

## Quick start

A ROS 2 launch file has been provided to start the Alpha 5 driver. To launch the
driver using default arguments, run

```bash
ros2 launch alpha_bringup alpha.launch.py
```

A full description of the launch arguments and their respective default values
can be obtained by running the following command:

```bash
ros2 launch alpha_bringup alpha.launch.py --show-args
```

## Getting help

If you have questions regarding usage of the Alpha 5 driver or regarding
contributing to this project, please ask a question on our
[Discussions](https://github.com/evan-palmer/alpha/discussions) board!

## License

Any proprietary documents or software owned by Reach Robotics and used within
this project are made available with official permission from Reach
Robotics and have been documented accordingly. All other software is is released
under the MIT license.
