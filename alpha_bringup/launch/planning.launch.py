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
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """
    Generate a launch description to run MoveIt with the Reach Alpha 5 manipulator.

    Returns:
        LaunchDescription: ROS2 launch description
    """
    # Declare arguments
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
            "prefix",
            default_value="''",
            description=(
                "Prefix of the joint names; useful for multi-robot setup. If"
                " changed, then the joint names in the controller configuration must"
                " be updated. Expected format '<prefix>/'"
            ),
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Automatically start RViz2",
        ),
        DeclareLaunchArgument(
            "use_sim",
            default_value="false",
            description="Automatically start Gazebo",
        ),
    ]

    # Initialize Arguments
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    prefix = LaunchConfiguration("prefix")
    use_rviz = LaunchConfiguration("use_rviz")
    use_sim = LaunchConfiguration("use_sim")

    robot_description = {
        "robot_description": Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [FindPackageShare(description_package), "config", description_file]
                ),
                " ",
                "prefix:=",
                prefix,
            ]
        )
    }

    robot_description_semantic = {
        "robot_description_semantic": Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [
                        FindPackageShare(description_package),
                        "srdf",
                        "alpha.config.srdf.xacro",
                    ]
                ),
                " ",
                "prefix:=",
                prefix,
                " ",
                "description_package:=",
                description_package,
            ]
        )
    }

    # Load the moveit configuration files
    robot_description_planning_joint_limits = PathJoinSubstitution(
        [
            FindPackageShare(description_package),
            "moveit2",
            "joint_limits.yaml",
        ]
    )

    robot_description_planning_cartesian_limits = PathJoinSubstitution(
        [
            FindPackageShare(description_package),
            "moveit2",
            "pilz_cartesian_limits.yaml",
        ]
    )

    robot_description_kinematics = PathJoinSubstitution(
        [FindPackageShare(description_package), "moveit2", "kinematics.yaml"]
    )

    moveit_controllers = PathJoinSubstitution(
        [
            FindPackageShare(description_package),
            "moveit2",
            "moveit_controllers.yaml",
        ]
    )

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning_cartesian_limits,
            robot_description_planning_joint_limits,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {"use_sim_time": use_sim},
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "rviz", "moveit.rviz"]
            ),
        ],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_planning_cartesian_limits,
            robot_description_planning_joint_limits,
            robot_description_kinematics,
        ],
        condition=IfCondition(use_rviz),
    )

    nodes = [
        move_group_node,
        rviz_node,
    ]

    return LaunchDescription(args + nodes)
