from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the launch directory
    package_name = 'ros2_nmpc'
    package_dir = get_package_share_directory(package_name)
    
    # Declare the launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(package_dir, 'config', 'nmpc_params_diff.yaml'),
        description='Path to the YAML configuration file'
    )

    # Create the node
    nmpc_node = Node(
        package=package_name,
        executable='nmpc_differential_drive_node',
        name='nmpc_differential_drive',
        output='screen',
        parameters=[LaunchConfiguration('config_file')]
    )

    return LaunchDescription([
        config_file_arg,
        nmpc_node
    ])