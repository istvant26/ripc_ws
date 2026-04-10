import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # --- PATHS ---
    pkg_localization = get_package_share_directory('ripc_localization')

    # 1. RTCM Relay (Using remapping or arguments instead of parameters)
    rtcm_relay_node = Node(
        package='topic_tools',
        executable='relay',
        name='rtcm_relay',
        # Relay usually takes positional arguments for input and output
        arguments=['/gps_stern/moving_base_rtcm', '/rtcm_moving_base']
    )

    land_base_node = Node(
        package='ublox_gps',
        executable='ublox_gps_node',
        name='land_base_node',
        output='screen',
        parameters=[{
            'device': '/dev/serial/by-path/pci-0000:02:00.0-usb-0:5.1:1.0',
            'uart1/baudrate': 115200,
            'frame_id': 'land_station',
            'gnss/rtcm': True,
            'tmode3': 1,
            'sv_in/min_dur': 60,
            'sv_in/acc_lim': 2000,
            'cfg/set': False,
            'auto_reconnect': True
        }],
        remappings=[
            ('/rtcm', '/rtcm_land')      # Remap the standard output to your topic
        ]
    )

    # 3. Include your existing EKF launch file
    included_ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_localization, 'launch', 'dual_ekf.launch.py')
        )
    )

    return LaunchDescription([
        included_ekf_launch,
        rtcm_relay_node,
        land_base_node
    ])