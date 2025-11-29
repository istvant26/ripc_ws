from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    # Path to your cylinder SDF
    cylinder_sdf = '/home/riplab/ripc_ws/src/my_custom_sim/my_usv_description/models/my_cylinder/model.sdf'

    return LaunchDescription([
        # 1️⃣ Launch the Sydney Regatta world
        ExecuteProcess(
            cmd=[
                'ros2', 'launch', 'vrx_gz', 'sydney_regatta.launch.py'
            ],
            output='screen'
        ),

        # 2️⃣ Spawn the cylinder after a short delay (wait for Gazebo to start)
        Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_my_cylinder',
            arguments=[
                '-file', cylinder_sdf,
                '-name', 'my_cylinder',
                '-x', '0',   # Adjust relative to WAM-V
                '-y', '5',   # 5 meters in front
                '-z', '0.5', # half meter above water
                '-world', 'sydney_regatta'
            ],
            output='screen'
        )
    ])
