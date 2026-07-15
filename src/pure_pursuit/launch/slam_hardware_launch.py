import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """SLAM on the car (Lab 5 deliverable 1).

    Bring up the base stack first (VESC odom -> base_link TF + lidar), then
    this, then joystick-drive 2 slow laps of Levine and save:
      ros2 run nav2_map_server map_saver_cli -f levine_2nd
    """
    config = os.path.join(
        get_package_share_directory('pure_pursuit'),
        'config', 'slam_toolbox_hardware.yaml')

    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[config],
            output='screen',
        )
    ])
