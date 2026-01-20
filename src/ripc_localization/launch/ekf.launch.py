import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Setup paths to your config files
    pkg_share = get_package_share_directory('ripc_localization')
    
    # Ensure these filenames match your actual files in the config folder
    navsat_params = os.path.join(pkg_share, 'config', 'navsat_transform_node.yaml')
    ekf_params = os.path.join(pkg_share, 'config', 'ekf.yaml')

    # 2. Declare use_sim_time as a launch argument so it can be toggled easily
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        # NavSat Transform Node
        # This translates GPS (Lat/Long) into Odometry (X, Y meters)
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            output='screen',
            parameters=[
                navsat_params,
                {'use_sim_time': use_sim_time}
            ],
            remappings=[
                ('imu', '/model/ripc_usv/imu_bow'),
                ('gps/fix', '/gps/fix'),
                ('odometry/filtered', '/odometry/filtered'),
                ('odometry/gps', '/odometry/gps')
            ]
        ),

        # EKF Node
        # This fuses IMU and the GPS-generated Odometry
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                ekf_params,
                {'use_sim_time': use_sim_time}
            ],
            remappings=[
                ('odometry/filtered', '/odometry/filtered')
            ]
        ),
    ])