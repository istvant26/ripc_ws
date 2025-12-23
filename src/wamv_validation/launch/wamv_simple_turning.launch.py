from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
import time
import os

def generate_launch_description():

    output_bag_dir = LaunchConfiguration('output_bag_dir')

    # Generate unique bag path using normal Python
    timestamp = str(int(time.time()))
    unique_bag_path = os.path.join(
        '/home/riplab/ripc_ws/wamv_bags', f'wamv_bag_{timestamp}'
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'output_bag_dir',
            default_value='/home/riplab/ripc_ws/wamv_bags',
            description='Directory where rosbag output should be stored'
        ),

        # Example node
        Node(
            package='wamv_validation',
            executable='wamv_right_thruster_turn',
            name='wamv_right_thruster_turn',
            output='screen',
            parameters=[{
                'thrust': 100.0,
                'on_time': 16.25,
                'off_time': 10.0
            }],
        ),

        # Rosbag recording using unique file path
        ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'record',
                '-s', 'mcap',
                '/wamv/thrusters/left/thrust',
                '/wamv/thrusters/right/thrust',
                '/wamv/sensors/position/ground_truth_odometry',
                '-o', unique_bag_path
            ],
            output='screen'
        )

    ])
