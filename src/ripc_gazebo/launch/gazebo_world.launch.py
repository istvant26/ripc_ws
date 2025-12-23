from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('ripc_usv_gazebo')
    world = os.path.join(pkg_share, 'worlds', 'sydney_regatta_custom.sdf')

    # Start gzserver (classic Gazebo). If you use ign-gazebo, replace commands accordingly.
    gzserver = ExecuteProcess(
        cmd=['gzserver', '-s', 'libgazebo_ros_factory.so', world],
        output='screen'
    )

    # Start gzclient (GUI). Optional on headless machines.
    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )

    return LaunchDescription([
        gzserver,
        gzclient
    ])

