import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('wall_follow'),
        'config',
        'hardware_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='wall_follow',
            executable='wall_follow_node',
            name='wall_follow_node',
            parameters=[
                config,
                {'use_sim_time': False},
            ],
            # Remap if your LiDAR driver publishes to a different topic name.
            # Default /scan matches most ROS2 LiDAR drivers (rplidar, urg_node, etc.)
            remappings=[
                ('/scan', '/scan'),
                ('/drive', '/drive'),
            ],
            output='screen',
        )
    ])
