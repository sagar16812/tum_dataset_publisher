from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os

def launch_rviz2(context):
    visualize = context.launch_configurations.get('visualize', 'false')
    if visualize.lower() == 'true':
        # Get the package share directory and construct the path to the RViz config file
        package_share_directory = get_package_share_directory('tum_dataset_publisher')
        rviz_config_path = os.path.join(package_share_directory, 'config', 'tum_dataset_publisher.rviz')

        return [ExecuteProcess(
            cmd=['rviz2', '-d', rviz_config_path],
            output='screen'
        )]
    return []

def generate_launch_description():
    dataset_type_arg = DeclareLaunchArgument('dataset_type', default_value='rgbd', description='Dataset type (mono, stereo, rgbd)')
    sequence_path_arg = DeclareLaunchArgument('sequence_path', description='Path to the TUM dataset sequence')
    frame_rate_arg = DeclareLaunchArgument('frame_rate', default_value='30', description='Frame rate for publishing images')
    groundtruth_arg = DeclareLaunchArgument('groundtruth', default_value='false', description='Publish groundtruth data')
    accelerometer_arg = DeclareLaunchArgument('accelerometer', default_value='false', description='Publish accelerometer data')
    visualize_arg = DeclareLaunchArgument('visualize', default_value='false', description='Launch RViz2 with saved configuration')

    tum_publisher_node = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'tum_dataset_publisher', 'tum_publisher_node',
            LaunchConfiguration('dataset_type'),
            LaunchConfiguration('sequence_path'),
            LaunchConfiguration('frame_rate'),
            LaunchConfiguration('groundtruth'),
            LaunchConfiguration('accelerometer'),
            LaunchConfiguration('visualize')
        ],
        output='screen'
    )

    return LaunchDescription([
        dataset_type_arg,
        sequence_path_arg,
        frame_rate_arg,
        groundtruth_arg,
        accelerometer_arg,
        visualize_arg,
        tum_publisher_node,
        OpaqueFunction(function=launch_rviz2)
    ])