import os
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # --- PATHS ---
    pkg_share = get_package_share_directory('ripc_localization')
    # Path to your Xacro file (adjust package name if needed)
    xacro_file = os.path.join(get_package_share_directory('ripc_description'), 'urdf', 'ripc_usv.urdf.xacro')
    
    ekf_local_params = os.path.join(pkg_share, 'config', 'dual_local_ekf.yaml')
    ekf_global_params = os.path.join(pkg_share, 'config', 'dual_global_ekf.yaml')
    navsat_params = os.path.join(pkg_share, 'config', 'navsat_transform.yaml')

    # --- PROCESS XACRO ---
    robot_description_config = xacro.process_file(xacro_file)
    robot_desc = robot_description_config.toxml()

    return LaunchDescription([
       
        # 2. EKF Local (Odom -> Base Link)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_local',
            output='screen',
            parameters=[ekf_local_params],
            remappings=[
                ('odometry/filtered', 'odometry/local'),
                ('imu0', '/imu_bow/data'),
                ('imu1', '/imu_stern/data'), # Added second IMU
                ('twist0', '/gps_bow/velocity'),
                ('twist1', '/gps_stern/velocity') # Added second Velocity source
            ]
        ),

        # 3. EKF Global (Map -> Odom)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_global',
            output='screen',
            parameters=[ekf_global_params],
            remappings=[
                ('odometry/filtered', 'odometry/global'),
                ('odom0', '/odometry/gps_bow'), 
                ('odom1', '/odometry/gps_stern'), # Added second GPS Odom
                ('imu0', '/imu_bow/data'),
                ('imu1', '/imu_stern/data'),
                ('twist0', '/gps_bow/velocity'),
                ('twist1', '/gps_stern/velocity')
            ]
        ),

        # 4. Navsat Transform - BOW
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_bow',
            output='screen',
            parameters=[navsat_params],
            remappings=[
                ('imu', '/imu_bow/data'),
                ('gps/fix', '/gps_bow/fix'),
                ('odometry/filtered', 'odometry/global'),
                ('odometry/gps', 'odometry/gps_bow'),
                ('gps/filtered', 'gps_bow/filtered')
            ]
        ),

        # 5. Navsat Transform - STERN
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_stern',
            output='screen',
            parameters=[navsat_params],
            remappings=[
                ('imu', '/imu_stern/data'),
                ('gps/fix', '/gps_stern/fix'),
                ('odometry/filtered', 'odometry/global'),
                ('odometry/gps', 'odometry/gps_stern'),
                ('gps/filtered', 'gps_stern/filtered')
            ]
        )
    ])