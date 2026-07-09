import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('gap_follow'),
        'config',
        'hardware_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='gap_follow',
            executable='reactive_node',
            name='reactive_node',
            parameters=[
                config,
                {'use_sim_time': False},
            ],
            # Output goes to /drive_gap_follow so ackermann_mux can arbitrate
            # priority against the safety node before forwarding to /drive.
            remappings=[
                ('/scan', '/scan'),
                ('/drive', '/drive_gap_follow'),
            ],
            output='screen',
        )
    ])
