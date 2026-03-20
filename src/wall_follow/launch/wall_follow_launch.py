from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wall_follow',
            executable='wall_follow_node',
            name='wall_follow_node',
            parameters=[
                {'kp': 0.4},
                {'ki': 0.0},
                {'kd': 0.1},
                {'desired_distance': 1.0},
                {'lookahead_L': 0.5},
                {'theta': 45.0},
            ],
            output='screen'
        )
    ])
