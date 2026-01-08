from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    urdf_path = os.path.join(
        get_package_share_directory('ripc_description'),
        'urdf',
        'ripc_usv.urdf.xacro'
    )
    
    # Converts the .urdf.xacro file to a urdf output for GZ to use
    robot_description = ParameterValue(
        Command(['xacro ' + urdf_path]),
        value_type=str
    )

    return LaunchDescription([

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher'
        ),

       
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen'
        )
    ])
