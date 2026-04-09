from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_share = get_package_share_directory('ripc_localization')
    ekf_local_params = os.path.join(pkg_share, 'config', 'single_local_ekf.yaml')
    ekf_global_params = os.path.join(pkg_share, 'config', 'single_global_ekf.yaml')
    navsat_params = os.path.join(pkg_share, 'config', 'navsat_transform.yaml')


        
    # --- NODE DEFINITIONS ---
    return LaunchDescription([
    Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_imu',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_gps',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'gps_link']
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_local',
            output='screen',
            parameters=[ekf_local_params],
            remappings=[
                ('odometry/filtered', 'odometry/local'),
                ('imu0', '/imu_bow/data'),
                ]
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_global',
            output='screen',
            parameters=[ekf_global_params],
            remappings=[
                ('odometry/filtered', 'odometry/global'),
                ('odom0', '/odometry/gps_bow'), 
                ('imu0', '/imu_bow/data'),
                ]
        ),
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=[navsat_params],
            remappings=[
                ('imu', '/imu_bow/data'),
                ('gps/fix', '/gps_bow/fix'),
                ('odometry/filtered', 'odometry/global'),
                ('odometry/gps', 'odometry/gps_bow'), # Change the output name here
                ('gps/filtered', 'gps_bow/filtered')
            ]
        )
    ])



