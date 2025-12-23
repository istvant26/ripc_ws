from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    ripc_gazebo_launch = PathJoinSubstitution([
        get_package_share_directory('ripc_gazebo'),
        'launch',
        'spawn_usv.launch.py',
        'gazebo_world.launch.py'
    ])


