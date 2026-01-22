from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction, DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_gazebo = get_package_share_directory('ripc_gazebo')
    pkg_description = get_package_share_directory('ripc_description')
    
    bridge_config = os.path.join(pkg_gazebo, 'config', 'bridge_config.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    urdf_path = os.path.join(pkg_description, 'urdf', 'ripc_usv.urdf.xacro')
    robot_description_content = ParameterValue(
        Command(['xacro ', urdf_path, ' use_sim_time:=', use_sim_time]),
        value_type=str
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        # 1. VRX Environment
        ExecuteProcess(
            cmd=['ros2', 'launch', 'vrx_gz', 'vrx_environment.launch.py', 
                 'world:=sydney_regatta', 'use_sim_time:=true'],
            output='screen'
        ),

        # 2. Bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='bridge_node',
            parameters=[{'config_file': bridge_config, 'use_sim_time': True, 'lazy': False}],
            output='screen'
        ),

        # 3. Robot State Publisher
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    parameters=[{'robot_description': robot_description_content, 'use_sim_time': True}],
                    output='screen'
                )
            ]
        ),

        # 4. Spawn USV
        TimerAction(
            period=8.0,
            actions=[
                ExecuteProcess(
                     cmd=['ros2', 'launch', 'ripc_gazebo', 'spawn_usv.launch.py', 'use_sim_time:=true'],
                     output='screen'
                )
            ]
        ),

        # 5. LOCAL EKF (odom -> base_link)
        TimerAction(
            period=12.0,
            actions=[
                Node(
                    package='robot_localization',
                    executable='ekf_node',
                    name='ekf_filter_node_local',
                    remappings=[('odometry/filtered', '/odometry/filtered/local')],
                    parameters=[{
                        'use_sim_time': True,
                        'two_d_mode': True,
                        'publish_tf': True,
                        'world_frame': 'odom',
                        'odom_frame': 'odom',
                        'base_link_frame': 'base_link',
                        'imu0': '/model/ripc_usv/imu_bow',
                        'imu0_config': [False, False, False,    # Position
                                        True, True, True,     # Orientation 
                                        False, False, False,    # Velocity 
                                        True, True, True,     # Gyroscope
                                        False, False, False],   # Acceleromter
                        'imu0_relative': True,
                        'imu0_qos': 'best_effort'
                    }]
                )
            ]
        ),

        # 6. GLOBAL EKF (map -> odom)
        TimerAction(
            period=15.0,
            actions=[
                Node(
                    package='robot_localization',
                    executable='ekf_node',
                    name='ekf_filter_node_global',
                    remappings=[('odometry/filtered', '/odometry/filtered/global')],
                    parameters=[{
                        'use_sim_time': True,
                        'frequency': 10.0,
                        'two_d_mode': True,
                        'publish_tf': True,      
                        'world_frame': 'map',
                        'map_frame': 'map',
                        'odom_frame': 'odom',
                        'base_link_frame': 'base_link',
                        'odom0': '/odometry/gps',
                        'odom0_config': [True, True, False,
                                         False, False, False, 
                                         False, False, False, 
                                         False, False, False, 
                                         False, False, False],
                        'imu0': '/model/ripc_usv/imu_bow',
                        'imu0_config': [False, False, False,    # Position
                                        True, True, True,     # Orientation 
                                        False, False, False,    # Velocity 
                                        True, True, True,     # Gyroscope
                                        True, True, True],   # Acceleromter
                        'imu0_relative': False, 
                        'imu0_qos':'best_effort'
                    }]
                )
            ]
        ),

        # 7. NAVSAT TRANSFORM
        TimerAction(    
            period=20.0,
            actions=[
                Node(
                    package='robot_localization',
                    executable='navsat_transform_node',
                    name='navsat_transform_node',
                    remappings=[
                        ('imu','/model/ripc_usv/imu_bow'),   
                        ('imu/data', '/model/ripc_usv/imu_bow'),
                        ('gps/fix', '/gps/fix'),
                        ('odometry/gps', '/odometry/gps'),
                        ('odometry/filtered', '/odometry/filtered/local')
                    ],
                    parameters=[{
                        'use_sim_time': True,
                        'wait_for_datum': True,
                        'datum': [-33.724223, 150.692, 0.0],
                        'yaw_offset': 0.0, # Set back to 1.5708 (standard for ENU)
                        'use_odometry_yaw': True, 
                        'publish_filtered_gps': True,
                        'broadcast_cartesian_transform': False, 
                        'map_frame': 'map',
                        'odom_frame': 'odom',
                        'base_link_frame': 'base_link',
                        'world_frame': 'map', # CRITICAL: MUST BE MAP
                    }]
                )
            ]
        ),

        # 8. RViz2
        TimerAction(
            period=25.0,
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