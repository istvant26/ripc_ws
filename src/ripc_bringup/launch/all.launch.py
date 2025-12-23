from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # package share directories
    control_pkg = get_package_share_directory('custom_usv_control')
    ekf_pkg = get_package_share_directory('usv_state_estimation')
    nav_pkg = get_package_share_directory('custom_usv_nav')
    mission_pkg = get_package_share_directory('custom_usv_mission')
    sensors_pkg = get_package_share_directory('custom_usv_sensors')

    # launch files
    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(sensors_pkg, 'launch', 'sensors.launch.py'))
    )

    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ekf_pkg, 'launch', 'ekf.launch.py'))
    )

    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(control_pkg, 'launch', 'control.launch.py'))
    )

    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav_pkg, 'launch', 'nav2.launch.py'))
    )

    mission_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(mission_pkg, 'launch', 'mission_planner.launch.py'))
    )

    # optional: delay Nav2 and Mission until EKF is ready
    nav_launch_delayed = TimerAction(period=5.0, actions=[nav_launch])
    mission_launch_delayed = TimerAction(period=6.0, actions=[mission_launch])

    return LaunchDescription([
        sensors_launch,
        ekf_launch,
        control_launch,
        nav_launch_delayed,
        mission_launch_delayed
    ])
