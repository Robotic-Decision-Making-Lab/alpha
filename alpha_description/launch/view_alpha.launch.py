# Copyright 2023, Evan Palmer
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """
    Generate a launch description to visualize the Alpha manipulator.

    Returns:
        LaunchDescription: ROS2 launch description
    """
    # Declare launch arguments
    args = [
        DeclareLaunchArgument(
            "description_package",
            default_value="alpha_description",
            description="Description package with the Alpha URDF files.",
        ),
        DeclareLaunchArgument(
            "description_file",
            default_value="alpha.urdf.xacro",
            description=(
                "URDF/XACRO description file with the Alpha; should be stored within a"
                " `urdf` directory"
            ),
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value="rviz.rviz",
            description=(
                "RViz configuration file; should be stored within a `rviz` directory"
            ),
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description=(
                "Use time data from the /clock topic instead of the system clock"
            ),
        ),
    ]

    # Initialize the arguments
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    rviz_config = LaunchConfiguration("rviz_config")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Declare ROS2 nodes
    nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "robot_description": ParameterValue(
                        Command(
                            [
                                PathJoinSubstitution([FindExecutable(name="xacro")]),
                                " ",
                                PathJoinSubstitution(
                                    [
                                        FindPackageShare(description_package),
                                        "urdf",
                                        description_file,
                                    ]
                                ),
                            ]
                        )
                    ),
                }
            ],
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            parameters=[],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="RVIZ",
            arguments=[
                "-d",
                PathJoinSubstitution(
                    [FindPackageShare(description_package), "rviz", rviz_config]
                ),
            ],
        ),
    ]

    return LaunchDescription(args + nodes)
