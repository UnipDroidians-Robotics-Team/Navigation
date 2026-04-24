import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    config_dir = os.path.join(get_package_share_directory('udh1_mapping'), 'config')
    map_config_file = os.path.join(config_dir, 'mapper_params_online_async.yaml')

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            map_config_file,
            {'use_sim_time': False}
        ]
    )

    return LaunchDescription([
        slam_node
    ])
