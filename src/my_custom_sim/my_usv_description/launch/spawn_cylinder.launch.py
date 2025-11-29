from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_my_cylinder',
            output='screen',
            arguments=[
                '-name', 'my_cylinder',
                '-file', '/home/riplab/ripc_ws/src/my_custom_sim/my_usv_description/models/my_cylinder/model.sdf',
                '-x', '0',    # position in front of WAM-V
                '-y', '2',    # adjust as needed
                '-z', '0',  # above water
                '-world', 'sydney_regatta'
            ]
        )
    ])
