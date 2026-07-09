from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Sim: defaults in reactive_node.cpp are the values validated in the
    # levine_blocked lap tests (7 laps / no contact). No overrides needed.
    return LaunchDescription([
        Node(
            package='gap_follow',
            executable='reactive_node',
            name='reactive_node',
            parameters=[{'use_sim_time': False}],
            output='screen',
        )
    ])
