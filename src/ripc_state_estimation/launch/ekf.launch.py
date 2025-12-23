from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('usv_state_estimation')
    ekf_config = os.path.join(pkg, 'config', 'ekf.yaml')
    navsat_config = os.path.join(pkg, 'config', 'navsat_transform.yaml')  # create this file next

    return LaunchDescription([
        # EKF fusing IMU + odometry
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config]
        ),

        # NavSat transform: GPS -> /odom -> /map
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            output='screen',
            parameters=[navsat_config]
        )
    ])
