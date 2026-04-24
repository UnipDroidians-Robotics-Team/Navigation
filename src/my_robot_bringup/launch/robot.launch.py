import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

class LidarConfig:
    PACKAGE_NAME = 'rplidar_ros'
    LAUNCH_FILE = 'rplidar_c1.launch.py'

    X_OFFSET = '0.1'
    Y_OFFSET = '0.0'
    Z_OFFSET = '0.0'
    YAW = '0.0'
    PITCH = '0.0'
    ROLL = '0.0'

    PARENT_FRAME = 'base_link'
    CHILD_FRAME = 'laser'  # Must match the frame ID published by the Lidar driver

def generate_launch_description():

    pkg_dir = get_package_share_directory(LidarConfig.PACKAGE_NAME)
    launch_file_path = os.path.join(pkg_dir, 'launch', LidarConfig.LAUNCH_FILE)

    lidar_launch_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file_path)
    )

    tf_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_static_tf_broadcaster',
        arguments=[
            '--x', LidarConfig.X_OFFSET,
            '--y', LidarConfig.Y_OFFSET,
            '--z', LidarConfig.Z_OFFSET,
            '--yaw', LidarConfig.YAW,
            '--pitch', LidarConfig.PITCH,
            '--roll', LidarConfig.ROLL,
            '--frame-id', LidarConfig.PARENT_FRAME,
            '--child-frame-id', LidarConfig.CHILD_FRAME
        ],
        output='screen'
    )

    return LaunchDescription([
        lidar_launch_node,
        tf_publisher_node
    ])
