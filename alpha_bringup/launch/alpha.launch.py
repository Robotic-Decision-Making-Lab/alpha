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
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """
    Generate a launch description to run the Reach Alpha 5 manipulator.

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
            default_value="alpha.config.xacro",
            description="URDF/XACRO description file with the Alpha",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value="view_alpha.rviz",
            description="RViz2 configuration file",
        ),
        DeclareLaunchArgument(
            "controllers_file",
            default_value="alpha_controllers.yaml",
            description="YAML file defining the ros2_control controllers",
        ),
        DeclareLaunchArgument(
            "initial_positions_file",
            default_value="initial_positions.yaml",
            description="Configuration file of robot initial positions for simulation.",
        ),
        DeclareLaunchArgument(
            "prefix",
            default_value="''",
            description=(
                "Prefix of the joint names; useful for multi-robot setup. If"
                " changed, then the joint names in the controller configuration must"
                " be updated. Expected format '<prefix>/'"
            ),
        ),
        DeclareLaunchArgument(
            "use_sim", default_value="false", description="Start the Gazebo simulation"
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Automatically start RViz2 with this launch file",
        ),
        DeclareLaunchArgument(
            "use_planning",
            default_value="false",
            description="Start the MoveIt2 interface",
        ),
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="true",
            description=(
                "Start robot with fake hardware mirroring command to its states."
            ),
        ),
        DeclareLaunchArgument(
            "serial_port",
            default_value="''",
            description="Serial port that the robot is available at",
        ),
        DeclareLaunchArgument(
            "timeout",
            default_value="5000",
            description=(
                "Maximum allowable time (ms) between heartbeat packets before a timeout"
                " warning is issued"
            ),
        ),
        DeclareLaunchArgument(
            "state_update_frequency",
            default_value="250",
            description=(
                "Frequency that the driver updates the state of the robot. Note that"
                " this should not be less than the read frequency defined by the "
                " controller configuration."
            ),
        ),
    ]

    # Initialize the arguments
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    rviz_config = LaunchConfiguration("rviz_config")
    controllers_file = LaunchConfiguration("controllers_file")
    initial_positions_file = LaunchConfiguration("initial_positions_file")
    prefix = LaunchConfiguration("prefix")
    use_sim = LaunchConfiguration("use_sim")
    use_rviz = LaunchConfiguration("use_rviz")
    use_planning = LaunchConfiguration("use_planning")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    serial_port = LaunchConfiguration("serial_port")
    timeout = LaunchConfiguration("timeout")
    state_update_frequency = LaunchConfiguration("state_update_frequency")

    robot_description = {
        "robot_description": Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [
                        FindPackageShare(description_package),
                        "config",
                        description_file,
                    ]
                ),
                " ",
                "prefix:=",
                " ",
                prefix,
                " ",
                "use_sim:=",
                use_sim,
                " ",
                "use_fake_hardware:=",
                use_fake_hardware,
                " ",
                "serial_port:=",
                " ",
                serial_port,
                " ",
                "hearbeat_timeout:=",
                timeout,
                " ",
                "state_update_frequency:=",
                state_update_frequency,
                " ",
                "controllers_file:=",
                controllers_file,
                " ",
                "initial_positions_file:=",
                initial_positions_file,
            ]
        )
    }

    # Declare ROS2 nodes
    nodes = [
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                robot_description,
                PathJoinSubstitution(
                    [
                        FindPackageShare(description_package),
                        "config",
                        controllers_file,
                    ]
                ),
            ],
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[robot_description],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="RVIZ2",
            arguments=[
                "-d",
                PathJoinSubstitution(
                    [FindPackageShare(description_package), rviz_config]
                ),
            ],
            parameters=[robot_description],
            condition=IfCondition(
                PythonExpression(
                    ["not ", use_planning, "and ", use_rviz]
                )  # launch either moveit or the alpha visualization
            ),
        ),
    ]

    return LaunchDescription(args + nodes)
