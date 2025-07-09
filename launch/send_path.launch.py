# launch/send_path.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    params = PathJoinSubstitution([
        get_package_share_directory('mrs_uav_path_loader'),
        'params', 'demo_path.yaml'
    ])

    return LaunchDescription([
        Node(
            package='mrs_uav_path_loader',
            executable='path_loader_node',
            namespace='uav1',                 
            remappings=[('path_out', 'trajectory_generation/path')],
            name='path_loader', 
            parameters=[params],
            output='screen'
        )
    ])
