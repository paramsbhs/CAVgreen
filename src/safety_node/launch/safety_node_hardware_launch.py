import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('safety_node'),
        'config',
        'hardware_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='safety_node',
            executable='safety_node',
            name='safety_node',
            parameters=[
                config,
                {'use_sim_time': False},
            ],
            remappings=[
                # safety_node.cpp subscribes to /ego_racecar/odom (sim topic).
                # On hardware, VESC publishes odometry to /odom.
                ('/ego_racecar/odom', '/odom'),
                ('/scan', '/scan'),
                # Output goes to /drive_safety so ackermann_mux gives it
                # priority 100 (beats wall_follow at priority 10).
                ('/drive', '/drive_safety'),
            ],
            output='screen',
        )
    ])
