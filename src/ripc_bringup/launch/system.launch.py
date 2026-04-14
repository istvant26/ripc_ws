import os
import xacro
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_localization = get_package_share_directory('ripc_localization')
    pkg_description = get_package_share_directory('ripc_description')
    
    xacro_file = os.path.join(pkg_description, 'urdf', 'ripc_usv.urdf.xacro')
    robot_description_raw = xacro.process_file(xacro_file).toxml()
    
    # REMOVED: land_base_gps (ublox_gps_node)
    # We're reading serial directly instead
    
    # Serial Bridge - reads /dev/ttyACM1, publishes /rtcm_land
    land_base_serial = Node(
        package='ripc_localization',
        executable='rtcm_bridge.py',  # Updated script below
        name='land_base_serial',
        output='screen'
    )
    
    # Dual EKF
    included_ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_localization, 'launch', 'dual_ekf.launch.py')
        )
    )
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_raw,
            'publish_frequency': 30.0
        }]
    )
    
    # Joint State Publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'robot_description': robot_description_raw}]
    )
    
    return LaunchDescription([
        land_base_serial,
        included_ekf_launch,
        robot_state_publisher,
        joint_state_publisher
    ])