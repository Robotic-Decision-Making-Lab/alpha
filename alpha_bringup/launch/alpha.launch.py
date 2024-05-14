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
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
        LaunchDescription: The Alpha 5 driver ROS 2 launch description.
    """
    # Declare launch arguments
    args = [
        DeclareLaunchArgument(
            "description_package",
            default_value="alpha_description",
            description=(
                "The description package with the Alpha URDF files. This is typically"
                " not set, but is available in case another description package has"
                " been defined."
            ),
        ),
        DeclareLaunchArgument(
            "description_file",
            default_value="alpha.config.xacro",
            description="The URDF/XACRO description file with the Alpha.",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value="view_alpha.rviz",
            description="The RViz2 configuration file.",
        ),
        DeclareLaunchArgument(
            "controllers_file",
            default_value="alpha_controllers.yaml",
            description="The YAML file defining the ros2_control controllers.",
        ),
        DeclareLaunchArgument(
            "initial_positions_file",
            default_value="initial_positions.yaml",
            description=(
                "The configuration file that defines the initial positions used for"
                " simulation."
            ),
        ),
        DeclareLaunchArgument(
            "initial_velocities_file",
            default_value="initial_velocities.yaml",
            description=(
                "The configuration file that defines the initial velocities used for"
                " simulation."
            ),
        ),
        DeclareLaunchArgument(
            "prefix",
            default_value="",
            description=(
                "The prefix of the joint names. This is useful for multi-robot setups."
                " If the prefix is changed, then the joint names in the controller"
                " configuration must be updated. Expected format '<prefix>/'."
            ),
        ),
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description=(
                "The namespace of the launched nodes. This is useful for multi-robot"
                " setups. If the namespace is changed, then the namespace in the"
                " controller configuration must be updated. Expected format '<ns>/'."
            ),
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Automatically start RViz2.",
        ),
        DeclareLaunchArgument(
            "use_planning",
            default_value="true",
            description="Automatically start the MoveIt2 interface.",
        ),
        DeclareLaunchArgument(
            "use_sim",
            default_value="true",
            description="Automatically start Gazebo.",
        ),
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="true",
            description=(
                "Start the driver using fake hardware mirroring command to its states."
                " If this is set to 'false', the Alpha 5 manipulator serial port should"
                " be specified."
            ),
        ),
        DeclareLaunchArgument(
            "serial_port",
            default_value="''",
            description=(
                "The serial port that the Alpha 5 is available at (e.g., /dev/ttyUSB0)."
            ),
        ),
        DeclareLaunchArgument(
            "state_update_frequency",
            default_value="250",
            description=(
                "The frequency (Hz) at which the driver updates the state of the robot."
                " Note that this should not be less than the read frequency defined by"
                " the controller configuration."
            ),
        ),
        DeclareLaunchArgument(
            "robot_controller",
            default_value="forward_position_controller",
            description=(
                "The ros2_control controller to use. When using MoveIt for planning,"
                " this argument will be ignored and the JointTrajectoryController will"
                " be used instead."
            ),
        ),
        DeclareLaunchArgument(
            "gazebo_world_file",
            default_value="empty.world",
            description="The world configuration to load if using Gazebo.",
        ),
    ]

    # Initialize the arguments
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    rviz_config = LaunchConfiguration("rviz_config")
    controllers_file = LaunchConfiguration("controllers_file")
    initial_positions_file = LaunchConfiguration("initial_positions_file")
    prefix = LaunchConfiguration("prefix")
    use_rviz = LaunchConfiguration("use_rviz")
    use_planning = LaunchConfiguration("use_planning")
    use_sim = LaunchConfiguration("use_sim")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    serial_port = LaunchConfiguration("serial_port")
    state_update_frequency = LaunchConfiguration("state_update_frequency")
    robot_controller = LaunchConfiguration("robot_controller")
    initial_velocities_file = LaunchConfiguration("initial_velocities_file")
    namespace = LaunchConfiguration("namespace")
    gazebo_world_file = LaunchConfiguration("gazebo_world_file")

    # Generate the robot description using xacro
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
                prefix,
                " ",
                "use_fake_hardware:=",
                use_fake_hardware,
                " ",
                "serial_port:=",
                serial_port,
                " ",
                "state_update_frequency:=",
                state_update_frequency,
                " ",
                "controllers_file:=",
                controllers_file,
                " ",
                "initial_positions_file:=",
                initial_positions_file,
                " ",
                "namespace:=",
                namespace,
                " ",
                "initial_velocities_file:=",
                initial_velocities_file,
                " ",
                "use_sim:=",
                use_sim,
            ]
        )
    }

    # Declare ROS 2 nodes
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        namespace=namespace,
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
        condition=UnlessCondition(use_sim),
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        namespace=namespace,
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            [namespace, "controller_manager"],
        ],
    )

    feedback_joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "feedback_joint_trajectory_controller",
            "--controller-manager",
            [namespace, "controller_manager"],
        ],
        condition=IfCondition(use_planning),
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            robot_controller,
            "--controller-manager",
            [namespace, "controller_manager"],
        ],
        condition=UnlessCondition(use_planning),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "rviz", rviz_config]
            ),
        ],
        parameters=[robot_description],
        condition=IfCondition(
            PythonExpression(
                ["'", use_planning, "' == 'false' and '", use_rviz, "' == 'true'"]
            )  # launch either MoveIt or the Alpha visualization, not both
        ),
    )

    gazebo_spawn_entity_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic",
            [namespace, "robot_description"],
            "-entity",
            [namespace, "alpha"],
        ],
        output="screen",
        condition=IfCondition(use_sim),
    )

    # Delay `joint_state_broadcaster` after control_node
    delay_joint_state_broadcaster_spawner_after_control_node = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=control_node,
            on_start=[joint_state_broadcaster_spawner],
        ),
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        ),
        condition=IfCondition(use_rviz),
    )

    # Delay start of feedback_joint_trajectory_controller after
    # `joint_state_broadcaster`
    delay_moveit_controller_spawners_after_joint_state_broadcaster_spawner = (
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[feedback_joint_trajectory_controller_spawner],
            ),
        )
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawners_after_joint_state_broadcaster_spawner = (
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[robot_controller_spawner],
            ),
        )
    )

    # Delay `joint_state_broadcaster` after spawn_entity
    delay_joint_state_broadcaster_spawner_after_spawn_entity = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gazebo_spawn_entity_node,
            on_exit=[joint_state_broadcaster_spawner],
        ),
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        gazebo_spawn_entity_node,
        delay_joint_state_broadcaster_spawner_after_control_node,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_moveit_controller_spawners_after_joint_state_broadcaster_spawner,
        delay_robot_controller_spawners_after_joint_state_broadcaster_spawner,
        delay_joint_state_broadcaster_spawner_after_spawn_entity,
    ]

    # Integrate additional launch files
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("alpha_bringup"), "/launch", "/planning.launch.py"]
        ),
        launch_arguments={
            "description_package": description_package,
            "description_file": description_file,
            "prefix": prefix,
            "namespace": namespace,
            "start_rviz": use_rviz,
        }.items(),
        condition=IfCondition(use_planning),
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"]
                )
            ]
        ),
        launch_arguments={
            "verbose": "true",
            "world": PathJoinSubstitution(
                [
                    FindPackageShare(description_package),
                    "gazebo",
                    "worlds",
                    gazebo_world_file,
                ]
            ),
        }.items(),
        condition=IfCondition(use_sim),
    )

    include = [moveit_launch, gazebo_launch]

    return LaunchDescription(args + include + nodes)
