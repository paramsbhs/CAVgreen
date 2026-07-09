import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share = get_package_share_directory('pure_pursuit')
    config = os.path.join(share, 'config', 'sim_params.yaml')
    default_csv = os.path.join(share, 'waypoints', 'levine_blocked_centerline.csv')

    return LaunchDescription([
        Node(
            package='pure_pursuit',
            executable='pure_pursuit_node.py',
            name='pure_pursuit_node',
            parameters=[
                config,
                {'use_sim_time': False},
                {'waypoint_csv': default_csv},
            ],
            # Sim: ground-truth pose from the gym bridge, direct drive output.
            remappings=[
                ('odom', '/ego_racecar/odom'),
                ('drive', '/drive'),
            ],
            output='screen',
        )
    ])
