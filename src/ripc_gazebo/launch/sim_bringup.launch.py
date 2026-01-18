from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    pkg_share = get_package_share_directory('ripc_gazebo')
    pkg_description = get_package_share_directory('ripc_description')

    pkg_share2 = get_package_share_directory('ripc_localization')
    ekf_config_path = os.path.join(pkg_share2, 'config', 'ekf.yaml')
    
    bridge_config = os.path.join(pkg_share, 'config', 'bridge_config.yaml')
    
    # 1. Process the URDF directly in the Master Launch
    urdf_path = os.path.join(pkg_description, 'urdf', 'ripc_usv.urdf.xacro')
    robot_description_content = ParameterValue(
        Command(['xacro ', urdf_path]),
        value_type=str
    )

    return LaunchDescription([

        # Launch VRX Sydney world
        ExecuteProcess(
            cmd=['ros2', 'launch', 'vrx_gz', 'vrx_environment.launch.py', 'world:=sydney_regatta'],
            output='screen'
        ),

        # 2. Launch Robot State Publisher DIRECTLY (No ExecuteProcess/Timer needed for the node itself)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description_content,
                'use_sim_time': True,
                'publish_frequency': 50.0,
                'ignore_timestamp': False,
            }],
            output='screen'
        ),
        
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),

        # 3. Launch RViz2 (Keep the timer if you want it to pop up later)
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    parameters=[{'use_sim_time': True}],
                    output='screen'
                )
            ]
        ),

        # 4. Spawn USV in Gazebo
        TimerAction(
            period=10.0,
            actions=[
                ExecuteProcess(
                     cmd=['ros2', 'launch', 'ripc_gazebo', 'spawn_usv.launch.py'],
                     output='screen'
                )
            ]
        ),

        # 5. The Bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='bridge_node',
            parameters=[{
                'config_file': bridge_config,
                'use_sim_time': True
            }],
            output='screen'
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config_path, {'use_sim_time': True}]
        ),
        
    ])