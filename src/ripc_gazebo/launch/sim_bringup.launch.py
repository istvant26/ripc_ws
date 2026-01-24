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
        # Keeps internal movements smooth. Accel is OFF to prevent quadratic drift.
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

                        'odom0': '/odometry/gps',
                        'odom0_config': [False, False, False,
                                         False, False, False, 
                                         True, True, False, 
                                         False, False, False, 
                                         False, False, False],

                        'imu0': '/model/ripc_usv/imu_bow',
                        'imu0_config': [False, False, False,    
                                        True, True, True,     
                                        False, False, False,    
                                        True, True, True,     
                                        False, False, False],   
                        'imu0_relative': True,
                        'imu0_qos': 'best_effort',

                        'imu1': '/model/ripc_usv/imu_stern',
                        'imu1_config': [False, False, False,    
                                        True, True , True,     
                                        False, False, False,    
                                        True, True, True,     
                                        False, False, False],   
                        'imu1_relative': True, 
                        'imu1_qos':'best_effort'
                    }]
                )
            ]
        ),

        # 6. GLOBAL EKF (map -> odom)
        # GPS is fused here. Accel is OFF to ensure GPS "Truth" wins.
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
                        'odom0_differential': True, # Prevents the "4 million meter" teleport

                        'imu0': '/model/ripc_usv/imu_bow',
                        'imu0_config': [False, False, False,    
                                        True, True, True,     
                                        False, False, False,    
                                        True, True, True,     
                                        False, False, False],   
                        'imu0_relative': False, 
                        'imu0_qos':'best_effort',

                        'imu1': '/model/ripc_usv/imu_stern',
                        'imu1_config': [False, False, False,    
                                        False, False , False,     
                                        False, False, False,    
                                        False, False, False,     
                                        False, False, False],   
                        'imu1_relative': False, 
                        'imu1_qos':'best_effort'
                    }]
                )
            ]
        ),

        # 7. NAVSAT TRANSFORM
        # Converts Lat/Long to X/Y. Alignment (yaw_offset) is critical here.
        TimerAction(    
            period=20.0,
            actions=[
                Node(
                    package='robot_localization',
                    executable='navsat_transform_node',
                    name='navsat_transform_node',
                    remappings=[
                        ('imu','/model/ripc_usv/imu_bow'),   
                        ('gps/fix', '/gps/fix'),
                        ('odometry/gps', '/odometry/gps'),
                        ('odometry/filtered', '/odometry/filtered/local')
                    ],
                    parameters=[{
                        'use_sim_time': True,
                        'wait_for_datum': False, # Let it pick start point as origin
                        'yaw_offset': 1.5708,     # Align Gazebo North (Y) with ENU North
                        'magnetic_declination_radians': 0.0,
                        'use_odometry_yaw': False, 
                        'publish_filtered_gps': True,
                        'zero_altitude': True,
                        'map_frame': 'map',
                        'odom_frame': 'odom',
                        'base_link_frame': 'base_link',
                        'world_frame': 'map',
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