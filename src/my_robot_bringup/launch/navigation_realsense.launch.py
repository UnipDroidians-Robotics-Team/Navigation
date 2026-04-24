import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    udh1_mapping_dir     = get_package_share_directory('udh1_mapping')
    udh1_description_dir = get_package_share_directory('udh1_description')
    my_bringup_dir       = get_package_share_directory('my_robot_bringup')

    map_file_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(
            os.path.expanduser('~'), 'athome_ws', 'mapas', 'mapa_udh1.yaml'
        ),
        description='Caminho para o .yaml do mapa'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(udh1_mapping_dir, 'config', 'nav2_params_realsense.yaml'),
        description='Parâmetros do Nav2 com RealSense D457'
    )

    map_file    = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')

    urdf_file = os.path.join(udh1_description_dir, 'urdf', 'udh1.urdf')
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False
        }]
    )

    base_driver_node = Node(
        package='serial_com_py',
        executable='base_driver_pulse',
        name='udh1_base_driver',
        output='screen'
    )

    lidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        output='screen',
        parameters=[{
            'channel_type':     'serial',
            'serial_port':      '/dev/ttyUSB0',
            'serial_baudrate':  460800,
            'frame_id':         'laser_frame',
            'inverted':         False,
            'angle_compensate': True,
            'scan_mode':        'DenseBoost'
        }]
    )

    filter_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(udh1_mapping_dir, 'launch', 'filter_launch.py')
        )
    )

    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        namespace='camera',
        output='screen',
        parameters=[{
            'enable_depth':  True,
            'enable_color':  False,
            'enable_infra1': False,
            'enable_infra2': False,
            'enable_accel':  False,
            'enable_gyro':   False,
            'depth_width':   640,
            'depth_height':  480,
            'depth_fps':     15,
            'pointcloud.enable':                  True,
            'pointcloud.stream_filter':           2,
            'spatial_filter.enable':              True,
            'temporal_filter.enable':             True,
            'decimation_filter.enable':           True,
            'decimation_filter.filter_magnitude': 2,
            'clip_distance':                      4.0,
            'base_frame_id':                      'camera_link',
            'depth_frame_id':                     'camera_depth_frame',
            'depth_optical_frame_id':             'camera_depth_optical_frame',
        }]
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time':  False,
            'yaml_filename': map_file
        }]
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file]
    )

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch', 'navigation_launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': 'false',        # nav2_bringup aceita string aqui
            'params_file':  params_file,
            'autostart':    'true',
        }.items()
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart':    True,
            'node_names':   ['map_server', 'amcl']
        }]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(my_bringup_dir, 'rviz', 'nav2_udh1.rviz')],
        parameters=[{'use_sim_time': False}]
    )

    return LaunchDescription([
        map_file_arg,
        params_file_arg,
        robot_state_publisher,
        base_driver_node,
        lidar_node,
        filter_launch,
        realsense_node,
        map_server_node,
        amcl_node,
        lifecycle_manager,
        nav2_bringup,
        rviz_node,
    ])
