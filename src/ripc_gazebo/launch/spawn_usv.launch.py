from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'ripc_usv',
                '-topic', 'robot_description'
            ],
            output='screen'
        )
    ])
