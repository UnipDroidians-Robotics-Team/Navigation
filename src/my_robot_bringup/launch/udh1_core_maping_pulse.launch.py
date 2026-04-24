import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    urdf_pkg_dir = get_package_share_directory('udh1_description')
    urdf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(urdf_pkg_dir, 'launch', 'display.launch.py'))
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
        parameters=[{
            'channel_type': 'serial',
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 460800,
            'frame_id': 'laser_frame',
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'DenseBoost'
        }],
        output='screen'
    )

    # câmera removida para poupar CPU durante SLAM

    return LaunchDescription([
        urdf_launch,
        base_driver_node,
        lidar_node
    ])
