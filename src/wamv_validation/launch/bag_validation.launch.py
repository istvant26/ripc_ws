from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():

    output_bag_dir = LaunchConfiguration('output_bag_dir')

    return LaunchDescription([

        # Argument for choosing the output directory
        DeclareLaunchArgument(
            'output_bag_dir',
            default_value='/home/riplab/ripc_ws/wamv_bags',
            description='Directory where rosbag output should be stored'
        ),

        # Compute twist from pose (your python node)
        Node(
            package='wamv_validation',
            executable='wamv_twist_from_pose',
            name='pose_to_twist',
            output='screen'
        ),

        # Start recording the bag
        ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'record',
                '/tf',
                '/tf_static',
                '/wamv/thrusters/left/thrust',
                '/wamv/thrusters/right/thrust',
                '/wamv/estimated_twist',
                '/wamv/sensors/imu/imu/data',
                '/wamv/sensors/gps/gps/fix',
                '-o', output_bag_dir
            ],
            output='screen'
        )
    ])
