from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_share = get_package_share_directory('ripc_gazebo')
    bridge_config = os.path.join(pkg_share, 'config', 'bridge_config.yaml')

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
                     cmd=['ros2', 'launch', 'ripc_description', 'description.launch.py','use_sim_time:=true'],
                     output='screen'
                )
            ]
        ),

        # Wait 10 seconds then display USV in RViz2
        TimerAction(
            period=10.0,
            actions=[
                ExecuteProcess(
                     cmd=['ros2', 'launch', 'ripc_gazebo', 'spawn_usv.launch.py'],
                     output='screen'
                )
            ]
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='bridge_node',
            parameters=[{'config_file': bridge_config}],
            output='screen'
        )

    ])
