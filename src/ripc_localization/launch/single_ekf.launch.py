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
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_bow']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_gps',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'gps_bow']
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_local',
            output='screen',
            parameters=[ekf_local_params],
            remappings=[
                ('odometry/filtered', 'odometry/local'),
<<<<<<< HEAD
                ('odom0', '/odometry/gps_bow'), 
                ('imu0', '/imu_bow/data'),
                ('twist0', '/gps_bow/velocity')
=======
>>>>>>> 363459e (Upload local workspace files)
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
<<<<<<< HEAD
                ('odom0', '/odometry/gps_bow'), 
                ('imu0', '/imu_bow/data'),
                ('twist0', '/gps_bow/velocity')
=======
>>>>>>> 363459e (Upload local workspace files)
                ]
        ),
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=[navsat_params],
            remappings=[
<<<<<<< HEAD
                ('imu', '/imu_bow/data'),
                ('gps/fix', '/gps_bow/fix'),
=======
>>>>>>> 363459e (Upload local workspace files)
                ('odometry/filtered', 'odometry/global'),
                ('odometry/gps', 'odometry/gps_bow'), # Change the output name here
                ('gps/filtered', 'gps_bow/filtered')
            ]
        )
    ])



