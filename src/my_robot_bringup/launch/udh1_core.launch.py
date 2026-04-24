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
        executable='base_driver',
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
            'scan_mode': 'DenseBoost'  # obrigatório para o C1
        }],
        output='screen'
    )

    realsense_pkg = get_package_share_directory('realsense2_camera')
    realsense_launch_file = os.path.join(realsense_pkg, 'launch', 'rs_launch.py')

    camera_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch_file),
        launch_arguments={
            'camera_name': 'camera',
            'align_depth.enable': 'true',
            'pointcloud.enable': 'true',
        }.items()
    )

    return LaunchDescription([
        urdf_launch,
        base_driver_node,
        lidar_node,
        camera_node
    ])
