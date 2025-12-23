from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='custom_usv_mission', executable='mission_planner_node.py', output='screen')
    ])
