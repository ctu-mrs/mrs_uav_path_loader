from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('mrs_uav_path_loader')
    params    = os.path.join(pkg_share, 'params', 'demo_path.yaml')

    return LaunchDescription([
        Node(
            package='mrs_uav_path_loader',
            executable='dummy_path_server',
            name='server'),
        Node(
            package='mrs_uav_path_loader',
            executable='path_loader_node',
            name='path_loader',
            parameters=[params]),
    ])
