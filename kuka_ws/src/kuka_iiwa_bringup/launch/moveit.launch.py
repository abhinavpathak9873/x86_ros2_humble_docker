import os
import yaml

from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def load_yaml(package_name: str, relative_path: str) -> dict:
    pkg_path = get_package_share_directory(package_name)
    abs_path = os.path.join(pkg_path, relative_path)
    with open(abs_path, "r") as f:
        return yaml.safe_load(f)


def load_file(package_name: str, relative_path: str) -> str:
    pkg_path = get_package_share_directory(package_name)
    abs_path = os.path.join(pkg_path, relative_path)
    with open(abs_path, "r") as f:
        return f.read()


def generate_launch_description():

    robot_description_content = Command([
        FindExecutable(name="xacro"), " ",
        PathJoinSubstitution([
            FindPackageShare("kuka_iiwa_description"), "urdf", "iiwa.urdf.xacro",
        ]),
        " mode:=mock",
        " robot_name:=iiwa",
    ])
    robot_description = {"robot_description": robot_description_content}

    robot_description_semantic = {
        "robot_description_semantic": load_file(
            "kuka_iiwa_moveit_config", "config/iiwa.srdf"
        )
    }

    robot_description_kinematics = {
        "robot_description_kinematics": load_yaml(
            "kuka_iiwa_moveit_config", "config/kinematics.yaml"
        )
    }

    robot_description_planning = {
        "robot_description_planning": load_yaml(
            "kuka_iiwa_moveit_config", "config/joint_limits.yaml"
        )
    }

    pipeline_yaml = load_yaml(
        "kuka_iiwa_moveit_config", "config/planning_pipeline.yaml"
    )
    planning_pipelines = {
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "ompl": pipeline_yaml.get("ompl", {}),
    }

    moveit_controllers = load_yaml(
        "kuka_iiwa_moveit_config", "config/moveit_controllers.yaml"
    )

    controllers_yaml = PathJoinSubstitution([
        FindPackageShare("kuka_iiwa_bringup"), "config", "iiwa_controllers.yaml",
    ])

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": False}],
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            planning_pipelines,
            moveit_controllers,
            {"use_sim_time": False},
            {"publish_robot_description_semantic": True},
            {"publish_planning_scene": True},
        ],
    )

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controllers_yaml],
        output="screen",
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

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", PathJoinSubstitution([
            FindPackageShare("kuka_iiwa_bringup"), "rviz", "iiwa_moveit.rviz",
        ])],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            {"use_sim_time": False},
        ],
    )

    return LaunchDescription([
        robot_state_publisher_node,
        controller_manager_node,
        move_group_node,
        joint_state_broadcaster_spawner,
        delay_arm_controller,
        rviz_node,
    ])