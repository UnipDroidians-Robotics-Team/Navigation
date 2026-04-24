import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    config_dir = os.path.join(get_package_share_directory('udh1_mapping'), 'config')
    filter_config = os.path.join(config_dir, 'laser_filters.yaml')

    laser_filter_node = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='scan_filter_chain',
        parameters=[filter_config],
        remappings=[
            ('/scan', '/scan'),
            ('/scan_filtered', '/scan_limpo')
        ]
    )

    return LaunchDescription([
        laser_filter_node
    ])
