import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share = get_package_share_directory('pure_pursuit')
    config = os.path.join(share, 'config', 'hardware_params.yaml')
    default_csv = os.path.join(share, 'waypoints', 'levine_2nd_recorded.csv')

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
            # Car: localization from particle_filter; drive goes through
            # ackermann_mux so safety_node keeps braking priority.
            remappings=[
                ('odom', '/pf/pose/odom'),
                ('drive', '/drive_pure_pursuit'),
            ],
            output='screen',
        )
    ])
