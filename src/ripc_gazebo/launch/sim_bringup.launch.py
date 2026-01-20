from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction, DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Paths
    pkg_gazebo = get_package_share_directory('ripc_gazebo')
    pkg_description = get_package_share_directory('ripc_description')
    pkg_localization = get_package_share_directory('ripc_localization')
    
    # Configs
    ekf_config_path = os.path.join(pkg_localization, 'config', 'ekf.yaml')
    navsat_config_path = os.path.join(os.path.dirname(get_package_share_directory('ripc_localization')), 'share', 'ripc_localization', 'config', 'navsat_transform_node.yaml')
    bridge_config = os.path.join(pkg_gazebo, 'config', 'bridge_config.yaml')
    
    # Substitutions
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # URDF Processing
    urdf_path = os.path.join(pkg_description, 'urdf', 'ripc_usv.urdf.xacro')
    robot_description_content = ParameterValue(
        Command(['xacro ', urdf_path, ' use_sim_time:=', use_sim_time]),
        value_type=str
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        # 1. VRX Environment (Launches Gazebo & Sydney World)
        ExecuteProcess(
            cmd=['ros2', 'launch', 'vrx_gz', 'vrx_environment.launch.py', 
                 'world:=sydney_regatta', 'use_sim_time:=true'],
            output='screen'
        ),

        # 2. Bridge (Must start early to provide /clock)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='bridge_node',
            parameters=[{
                'config_file': bridge_config,
                'use_sim_time': True,
                'lazy': False
            }],
            output='screen'
        ),

        # 3. Robot State Publisher (Delayed to ensure /clock is active)
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    parameters=[{
                        'robot_description': robot_description_content,
                        'use_sim_time': True,
                        'publish_frequency': 50.0
                    }],
                    output='screen'
                ),
                Node(
                    package='joint_state_publisher',
                    executable='joint_state_publisher',
                    parameters=[{'use_sim_time': True}],
                    output='screen'
                ),
            ]
        ),

        # 4. NavSat Transform (Converts GPS to Odometry)
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'broadcast_cartesian_transform_as_parent_frame': False,
                'publish_filtered_gps': True,
                'wait_for_datum': False,
                'datum': [-33.72422, 150.692, 0.0],
                'map_frame': 'map',
                'odom_frame': 'odom',
                'base_link_frame': 'base_link',
                'world_frame': 'map',
                # THIS IS THE CRITICAL FIX FOR THE QOS MISMATCH
                'qos_overrides./odometry/filtered.subscription.reliability': 'reliable'
            }],
            remappings=[
                ('imu', '/model/ripc_usv/imu_bow'),
                ('gps/fix', '/gps/fix'),
                ('odometry/gps', '/odometry/gps')
            ]
        ),

        # 5. EKF Filter (Fuses everything)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config_path, {'use_sim_time': True}]
        ),

        # 6. Spawn USV (Delayed until environment is ready)
        TimerAction(
            period=8.0,
            actions=[
                ExecuteProcess(
                     cmd=['ros2', 'launch', 'ripc_gazebo', 'spawn_usv.launch.py', 'use_sim_time:=true'],
                     output='screen'
                )
            ]
        ),

        # 7. RViz2
        TimerAction(
            period=12.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    arguments=['-d', os.path.join(pkg_description, 'rviz', 'config.rviz')],
                    parameters=[{'use_sim_time': True}],
                    output='screen'
                )
            ]
        ),
    ])