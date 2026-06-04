from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='airpost_drone', executable='dummy_camera', name='dummy_camera', output='screen'),
        Node(package='airpost_drone', executable='drone_node', name='airpost_drone', output='screen'),
    ])
