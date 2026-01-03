from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([

        # Bridge for thrusters
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='thruster_bridge',
            arguments=[
                '/ripc_usv/thrusters/left/thrust@std_msgs/msg/Float64@gz.msgs.Double',
                '/ripc_usv/thrusters/right/thrust@std_msgs/msg/Float64@gz.msgs.Double'
            ],
            output='screen'
        ),

        # Run Python script directly using system python
        ExecuteProcess(
            cmd=['python3', '/home/riplab/ripc_ws/src/ripc_control/scripts/send_thrust.py'],
            output='screen'
        ),

    ])
