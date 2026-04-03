import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Define the Node(s) you want to run
    control_node = Node(
        package='ripc_control',
        executable='stress_test_node.py', # Make sure this matches your setup.py
        name='ripc_Control',
        output='screen'
    )

    # 2. Return the LaunchDescription object
    return LaunchDescription([
        control_node
    ])