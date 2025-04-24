#-- The main launch file to trigger the nodes --#

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld=LaunchDescription()
    
    cns_node=Node(package="hardware_interface",executable="cns")
    
    sensorsuite_node=Node(package="hardware_interface",executable="sensorsuite")
    
    telemetry_node=Node(package="hardware_interface",executable="tele")
    
    camera_node=Node(package="hardware_interface",executable="camera_start")
    
    pose_node=Node(package="hardware_interface",executable="fall_detect")
    
    auto_cmd_node=Node(package="hardware_interface",executable="fall_cmd")
    
    ld.add_action(sensorsuite_node)
    ld.add_action(cns_node)
    ld.add_action(telemetry_node)
    ld.add_action(camera_node)
    ld.add_action(pose_node)
    ld.add_action(auto_cmd_node)
    
    return ld
