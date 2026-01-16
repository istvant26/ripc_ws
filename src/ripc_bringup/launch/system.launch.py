from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim = LaunchConfiguration('use_sim')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim', default_value='true'),

        # 1. SIMULATION MODE: Start the Bridges
        IncludeLaunchDescription(
            "path_to_ripc_gazebo/launch/bridge.launch.py",
            condition=IfCondition(use_sim)
        ),

        # 2. REAL MODE: Start Hardware Drivers
        IncludeLaunchDescription(
            "path_to_ripc_sensors/launch/hardware_drivers.launch.py",
            condition=UnlessCondition(use_sim)
        ),
    ])