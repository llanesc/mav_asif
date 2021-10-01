import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    params_file_path = os.path.join(get_package_share_directory(
        'mav_asif'), 'config', 'params.yaml')

    ld = LaunchDescription()

    mav1_asif_node = Node(
        package='mav_asif',
        executable='mav1_test',
        output='screen',
        parameters=[params_file_path],
    )

    ld.add_action(mav1_asif_node)
    return ld
