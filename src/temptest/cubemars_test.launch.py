
# from launch import LaunchDescription
# from launch_ros.actions import Node
# from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
# from launch_ros.substitutions import FindPackageShare


# def generate_launch_description():

#     # Path to the ros2_control xacro file
#     robot_description_file = PathJoinSubstitution(
#         [FindPackageShare("drive_controller"), "config", "cubemars_test.ros2_control.urdf.xacro"]
#     )

#     # Run xacro to generate robot_description parameter
#     robot_description = Command([
#         FindExecutable(name="xacro"),
#         " ",
#         robot_description_file
#     ])

#     # Controllers YAML
#     controller_config = PathJoinSubstitution(
#         [FindPackageShare("drive_controller"), "config", "controllers.yaml"]
#     )

#     # Robot State Publisher (standard even if we don't use TF yet)
#     robot_state_publisher = Node(
#         package="robot_state_publisher",
#         executable="robot_state_publisher",
#         parameters=[{"robot_description": robot_description}],
#         output="screen",
#     )

#     # ros2_control_node (controller manager)
#     controller_manager = Node(
#         package="controller_manager",
#         executable="ros2_control_node",
#         parameters=[{"robot_description": robot_description}, controller_config],
#         output="screen",
#     )

#     # Spawner for joint_state_broadcaster
#     joint_state_broadcaster_spawner = Node(
#         package="controller_manager",
#         executable="spawner",
#         arguments=["joint_state_broadcaster"],
#         output="screen",
#     )

#     # Spawner for forward_position_controller
#     forward_position_controller_spawner = Node(
#         package="controller_manager",
#         executable="spawner",
#         arguments=["forward_position_controller"],
#         output="screen",
#     )

#     return LaunchDescription([
#         robot_state_publisher,
#         controller_manager,
#         joint_state_broadcaster_spawner,
#         forward_position_controller_spawner,
#     ])

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Path to the ros2_control xacro file
    robot_description_file = PathJoinSubstitution(
        [FindPackageShare("drive_controller"), "urdf", "cubemars_test.ros2_control.urdf.xacro"]
    )

    # Run xacro to generate robot_description parameter
    robot_description = Command([
        FindExecutable(name="xacro"),
        " ",
        robot_description_file
    ])

    # Robot State Publisher (standard even if we don't use TF yet)
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen",
    )

    # Controllers YAML
    controller_config = PathJoinSubstitution(
        [FindPackageShare("drive_controller"), "config", "controllers.yaml"]
    )

    # ros2_control_node (controller manager)
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"robot_description": robot_description}, controller_config],
        output="screen",
    )

    # Spawner for joint_state_broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    # Spawner for drive_controller
    drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["drive_controller"],
        output="screen",
    )

    return LaunchDescription([
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        drive_controller_spawner,
    ])
