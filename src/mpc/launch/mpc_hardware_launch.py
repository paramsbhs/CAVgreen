import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share = get_package_share_directory('mpc')
    config = os.path.join(share, 'config', 'hardware_params.yaml')
    default_csv = os.path.join(
        get_package_share_directory('pure_pursuit'),
        'waypoints', 'levine_2nd_recorded.csv')

    return LaunchDescription([
        Node(
            package='mpc',
            executable='mpc_node.py',
            name='mpc_node',
            parameters=[
                config,
                {'use_sim_time': False},
                {'waypoint_csv': default_csv},
            ],
            # Car: localization from particle_filter; drive goes through
            # ackermann_mux so safety_node keeps braking priority.
            remappings=[
                ('odom', '/pf/pose/odom'),
                ('drive', '/drive_mpc'),
            ],
            output='screen',
        )
    ])
