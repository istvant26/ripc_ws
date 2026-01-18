from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 1. Declare the Argument (Defaulting to False for safety/real hardware)
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # 2. Reference the Argument value
    use_sim_time = LaunchConfiguration('use_sim_time')

    urdf_path = os.path.join(
        get_package_share_directory('ripc_description'),
        'urdf',
        'ripc_usv.urdf.xacro'
    )
    
    robot_description = ParameterValue(
        Command(['xacro ', urdf_path]),
        value_type=str
    )

    return LaunchDescription([
        use_sim_time_arg, # Add the declaration to the launch description

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True,
                'ignore_timestamp': True
            }],
            output='screen'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            parameters=[{
                'use_sim_time': True,
                'ignore_timestamp': True
            }],
            output='screen'
        )

    ])