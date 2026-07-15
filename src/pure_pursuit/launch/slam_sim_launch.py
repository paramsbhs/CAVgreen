import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """SLAM in the gym sim.

    Run alongside the bridge and something that drives the car (gap_follow or
    keyboard teleop). The map builds in the 'slam_map' frame; add a Map
    display for topic /map with fixed frame slam_map in RViz to watch it.
    Save when done:
      ros2 run nav2_map_server map_saver_cli -f <name> --ros-args -p map_subscribe_transient_local:=true
    """
    config = os.path.join(
        get_package_share_directory('pure_pursuit'),
        'config', 'slam_toolbox_sim.yaml')

    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[config],
            output='screen',
        )
    ])
