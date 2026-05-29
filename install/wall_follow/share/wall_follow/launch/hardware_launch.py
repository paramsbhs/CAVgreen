import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    wall_follow_share = get_package_share_directory('wall_follow')
    safety_node_share = get_package_share_directory('safety_node')

    mux_config = os.path.join(wall_follow_share, 'config', 'mux_params.yaml')

    wall_follow = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(wall_follow_share, 'launch', 'wall_follow_hardware_launch.py')
        )
    )

    safety_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(safety_node_share, 'launch', 'safety_node_hardware_launch.py')
        )
    )

    mux = Node(
        package='ackermann_mux',
        executable='ackermann_mux',
        name='ackermann_mux',
        parameters=[mux_config, {'use_sim_time': False}],
        remappings=[('/ackermann_cmd_out', '/drive')],
        output='screen',
    )

    return LaunchDescription([
        wall_follow,
        safety_node,
        mux,
    ])
