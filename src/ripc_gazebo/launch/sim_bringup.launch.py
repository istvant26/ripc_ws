from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
import os

def generate_launch_description():

    return LaunchDescription([

        # Launch VRX Sydney world using the VRX package
        ExecuteProcess(
            cmd=['ros2', 'launch', 'vrx_gz', 'vrx_environment.launch.py', 'world:=sydney_regatta'],
            output='screen'
        ),

        # Wait 5 seconds then display USV in RViz2
        TimerAction(
            period=5.0,
            actions=[
                ExecuteProcess(
                     cmd=['ros2', 'launch', 'ripc_description', 'description.launch.py'],
                     output='screen'
                )
            ]
        ),

        # Start ros_gz_bridge for thrusters
        TimerAction(
            period=6.0,  # slightly after spawn
            actions=[
                Node(
                    package='ros_gz_bridge',
                    executable='parameter_bridge',
                    name='thruster_bridge',
                    arguments=[
                        '/ripc_usv/thrusters/left/thrust@std_msgs/msg/Float64@gz.msgs.Double',
                        '/ripc_usv/thrusters/right/thrust@std_msgs/msg/Float64@gz.msgs.Double'
                    ],
                    output='screen'
                )
            ]
        ),

    ])
