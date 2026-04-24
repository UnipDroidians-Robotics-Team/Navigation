import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

class MapConfig:
    DEFAULT_MAP_PATH = os.path.expanduser('~/athome_ws/src/my_slam_maps/maps/meu_mapa_real.yaml')
    NODE_NAME_MAP_SERVER = 'map_server'
    NODE_NAME_LIFECYCLE = 'lifecycle_manager_map'
    ARG_NAME_MAP = 'map'
    ARG_DESC_MAP = 'Full path to the map yaml file'

def generate_launch_description():

    map_file_arg = DeclareLaunchArgument(
        MapConfig.ARG_NAME_MAP,
        default_value=MapConfig.DEFAULT_MAP_PATH,
        description=MapConfig.ARG_DESC_MAP
    )

    map_file = LaunchConfiguration(MapConfig.ARG_NAME_MAP)

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name=MapConfig.NODE_NAME_MAP_SERVER,
        output='screen',
        parameters=[{'yaml_filename': map_file}]
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name=MapConfig.NODE_NAME_LIFECYCLE,
        output='screen',
        parameters=[
            {'use_sim_time': False},
            {'autostart': True},
            {'node_names': [MapConfig.NODE_NAME_MAP_SERVER]}
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return LaunchDescription([
        map_file_arg,
        map_server_node,
        lifecycle_manager_node,
        rviz_node
    ])
