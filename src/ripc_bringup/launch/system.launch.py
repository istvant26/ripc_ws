import os
import xacro
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # --- PATHS ---
    pkg_localization = get_package_share_directory('ripc_localization')

    pkg_description = get_package_share_directory('ripc_description')
    xacro_file = os.path.join(pkg_description, 'urdf', 'ripc_usv.urdf.xacro')

    # NEW: Process the Xacro file into raw XML
    robot_description_config = xacro.process_file(xacro_file)
    robot_description_raw = robot_description_config.toxml()

    # 1. Land Base GPS Node
    # We configure this to output UInt8MultiArray directly to avoid the transformer crash
    land_base_node = Node(
        package='ublox_gps',
        executable='ublox_gps_node',
        name='land_base_node',
        output='screen',
       parameters=[{
            'device': '/dev/serial/by-path/pci-0000:00:14.0-usbv2-0:10.4:1.0',
            'uart1.baudrate': 115200,
            'frame_id': 'land_station',
            'gnss.rtcm': True,
            'tmode3': 2,                                      
            'arp.position': [400000000.0, -800000000.0, 0.0], 
            'arp.position_hp': [0, 0, 0],
            'arp.position_type': 0,                           
            
            # --- THE KEY FIXES ---
            'rtcm_type': 'std_msgs/UInt8MultiArray', 
            'publish': {
                'rxm': {
                    'rtcm': True,
                    'rtcm_type': 'std_msgs/UInt8MultiArray' # Set it inside the nested block too
                },
                'nav': {'svin': True}
            },
            # ---------------------

            'config_on_startup': True,
            'auto_reconnect': True
        }],
        remappings=[
            # REMOVE the leading slash here. 
            # This maps the node's local 'rxmrtcm' to the global '/rtcm_land'
            ('rxmrtcm', '/rxmrtcm_raw')
        ]
    )

    # 2. Include EKF launch file
    included_ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_localization, 'launch', 'dual_ekf.launch.py')
        )
    )

    # Use the verified path that just worked manually
    bridge_script = os.path.expanduser('~/ripc_ws/src/ripc_localization/nodes/rtcm_bridge.py')
    
    rtcm_bridge_process = ExecuteProcess(
        cmd=['python3', '-u', bridge_script], # Added -u for unbuffered logs
        output='screen',
        emulate_tty=True # This helps catch those hidden print statements
    )

    # 1. Robot State Publisher (REQUIRED for TF)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw,
                     'publish_frequency': 30.0,
                     'use_sim_time': False}]
    )

    # 2. Joint State Publisher (Now it knows WHICH joints to publish)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'robot_description': robot_description_raw}] # Give it the robot info
    )

    return LaunchDescription([
        included_ekf_launch,
        land_base_node,
        rtcm_bridge_process,
        robot_state_publisher,
        joint_state_publisher
    ])