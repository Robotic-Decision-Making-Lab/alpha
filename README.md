# Reach Alpha 5 ROS 2 Driver :mechanical_arm:

The Alpha 5 driver is a collection of ROS 2 packages designed
to support ROS 2 integration with the [Reach Alpha 5 manipulator](https://reachrobotics.com/products/manipulators/reach-alpha/).

## Main features

The main features of the Alpha 5 driver are:

- Integration of the [Reach System Serial Protocol](https://reach-robotics.github.io/reach_robotics_sdk/documentation/index.html#)
  for hardware communication
- `ros2_control` integration for manipulator position, velocity, and joint
  trajectory control
- Visualization support using RViz2
- Integration with MoveIt2 for motion planning
- Gazebo support for kinematic simulation

## Installation

The Alpha 5 driver is currently supported on Linux, and is available for the ROS
distributions Humble and Rolling. The driver can be installed from source. To
install the Alpha 5 driver from source, first clone this project to the `src`
directory of your ROS workspace

```bash
git clone -b $ROS_DISTRO git@github.com:evan-palmer/alpha.git
```

replacing `$ROS_DISTRO` with the desired ROS distribution or `main` for Rolling.
After cloning the project, install the ROS dependencies using `rosdep`, again
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

The Alpha 5 ROS 2 driver is released under the MIT license.
