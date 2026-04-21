from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    description_pkg = FindPackageShare("kuka_iiwa_description")
    bringup_pkg     = FindPackageShare("kuka_iiwa_bringup")

    robot_description_content = Command([
        FindExecutable(name="xacro"), " ",
        PathJoinSubstitution([description_pkg, "urdf", "iiwa.urdf.xacro"]),
        " mode:=mock",
        " robot_name:=iiwa",
    ])
    robot_description = {"robot_description": robot_description_content}

    controllers_yaml = PathJoinSubstitution([
        bringup_pkg, "config", "iiwa_controllers.yaml",
    ])

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controllers_yaml],
        output="screen",
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
        ],
        output="screen",
    )

    iiwa_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "iiwa_arm_controller",
            "--controller-manager", "/controller_manager",
        ],
        output="screen",
    )

    delay_arm_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[iiwa_arm_controller_spawner],
        )
    )

    return LaunchDescription([
        robot_state_publisher_node,
        controller_manager_node,
        joint_state_broadcaster_spawner,
        delay_arm_controller,
    ])