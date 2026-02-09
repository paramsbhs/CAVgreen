from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='safety_node',
            executable='safety_node',
            name='safety_node',
            parameters=[
                {'ttc_threshold': 0.5}
            ],
            output='screen'
        )
    ])
