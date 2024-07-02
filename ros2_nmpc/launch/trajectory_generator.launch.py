from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the path to the parameters file
    params_file = os.path.join(
        get_package_share_directory('ros2_nmpc'),
        'config',
        'trajectory_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='ros2_nmpc',
            executable='trajectory_node',
            name='trajectory_node',
            output='screen',
            parameters=[params_file]
        )
    ])