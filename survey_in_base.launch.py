import os
import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_localization = get_package_share_directory('ripc_localization')
    pkg_description = get_package_share_directory('ripc_description')

    xacro_file = os.path.join(pkg_description, 'urdf', 'ripc_usv.urdf.xacro')
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    ublox_params = os.path.join(pkg_localization, 'config', 'ripc_survey_in_base.yaml')
    rtcm_publisher_script = os.path.join(pkg_localization, 'nodes', 'rtcm_serial_publisher.py')

    rtcm_uart_port_arg = DeclareLaunchArgument(
        'rtcm_uart_port',
        default_value='/dev/ttyUSB0',
        description='USB-serial adapter connected to land-base F9P UART1 RTCM output'
    )

    rtcm_uart_baud_arg = DeclareLaunchArgument(
        'rtcm_uart_baud',
        default_value='115200',
        description='Baudrate for the land-base RTCM serial output'
    )

    land_base_node = Node(
        package='ublox_gps',
        executable='ublox_gps_node',
        name='land_base_node',
        output='screen',
        parameters=[ublox_params]
    )

    rtcm_serial_publisher = ExecuteProcess(
        cmd=[
            'python3',
            '-u',
            rtcm_publisher_script,
            '--port', LaunchConfiguration('rtcm_uart_port'),
            '--baudrate', LaunchConfiguration('rtcm_uart_baud'),
            '--topic', '/rtcm_land'
        ],
        output='screen',
        emulate_tty=True
    )

    included_ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_localization, 'launch', 'dual_ekf.launch.py')
        )
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_raw,
            'publish_frequency': 30.0,
            'use_sim_time': False
        }]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'robot_description': robot_description_raw
        }]
    )

    return LaunchDescription([
        rtcm_uart_port_arg,
        rtcm_uart_baud_arg,
        included_ekf_launch,
        land_base_node,
        rtcm_serial_publisher,
        robot_state_publisher,
        joint_state_publisher,
    ])
