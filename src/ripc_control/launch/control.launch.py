from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('custom_usv_control')
    cfg = os.path.join(pkg, 'config', 'usv_ros2_control.yaml')
    return LaunchDescription([
        Node(package='controller_manager', executable='ros2_control_node', output='screen', parameters=[cfg]),
        Node(package='controller_manager', executable='spawner', arguments=['thruster_controller']),
        Node(package='controller_manager', executable='spawner', arguments=['joint_state_broadcaster'])
    ])
