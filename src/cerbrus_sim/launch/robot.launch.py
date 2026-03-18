from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os
import sys


def generate_launch_description():
    pkg_path = get_package_share_directory('cerbrus_sim')
    
    return LaunchDescription([
        Node(
            package='cerbrus_sim',
            executable='sim_node.py',
            name='cerbrus_sim',
            output='screen',
            emulate_tty=True,  # Important for proper output
        ),
    ])