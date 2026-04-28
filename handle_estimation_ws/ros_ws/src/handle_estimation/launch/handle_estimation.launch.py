from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackagePrefix


def generate_launch_description():
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            '/opt/ros/humble/share/realsense2_camera/examples/pointcloud/rs_pointcloud_launch.py'
        ]),
        launch_arguments={
            'pointcloud.enable': 'true',
        }.items()
    )

    handle_estimation_node = Node(
        package='handle_estimation',
        executable='handle_pose_estimation_node',
        name='handle_pose_estimation',
        output='screen',
        parameters=[{
            'model_path': 'handler_model.pt',
            'confidence_threshold': 0.5,
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'model_path',
            default_value='handler_model.pt',
            description='Path to YOLO model'),
        DeclareLaunchArgument(
            'confidence_threshold',
            default_value='0.5',
            description='Detection confidence threshold'),
        realsense_launch,
        handle_estimation_node,
    ])