from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    
    output_bag_dir = LaunchConfiguration('output_bag_dir')

    return LaunchDescription([

        DeclareLaunchArgument(
            'output_bag_dir',
            default_value='/home/riplab/ripc_ws/wamv_bags',
            description='Directory where rosbag output should be stored'
        ),
       
        #WAM-V forward motion node
        Node(
           package='wamv_validation',
            executable='wamv_simple_forward',
            name='wamv_simple_forward',
            output='screen',
            parameters=[{
                'thrust': 100.0,   # Newtons
                'on_time': 16.25,   # seconds thrust ON
                'off_time': 10.0   # seconds thrust OFF
            }],
        ),
        

        # Start recording the bag
        ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'record',
                #'/wamv/pose',
                '/wamv/thrusters/left/thrust',
                '/wamv/thrusters/right/thrust',
                #'/wamv/sensors/imu/imu/data',
                #'/wamv/sensors/gps/gps/fix',
                #'/wamv/thrusters/left/thrust/ang_vel',
                #'/wamv/thrusters/right/thrust/ang_vel',
                '/wamv/sensors/position/ground_truth_odometry',
                '-o', output_bag_dir
            ],
            output='screen'
        )
      
    ])
