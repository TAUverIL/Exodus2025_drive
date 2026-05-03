from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get URDF via xacro
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("drive_controller"), "urdf", "cubemars_test.ros2_control.urdf.xacro"]
            ),
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # Controllers configuration
    robot_controllers = PathJoinSubstitution(
        [FindPackageShare("drive_controller"), "config", "controllers.yaml"]
    )

    # Controller Manager Node
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
    )

    # Joint State Broadcaster Spawner
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Drive Controller Spawner (start after joint_state_broadcaster)
    drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["drive_controller", "--controller-manager", "/controller_manager"],
    )

    # Pivot Controller Spawner (start after joint_state_broadcaster)
    pivot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["pivot_controller", "--controller-manager", "/controller_manager"],
    )

    # Delay start of controllers after joint_state_broadcaster
    delay_controllers_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[drive_controller_spawner, pivot_controller_spawner],
        )
    )

    nodes = [
        control_node,
        joint_state_broadcaster_spawner,
        delay_controllers_after_jsb,
    ]

    return LaunchDescription(nodes)
