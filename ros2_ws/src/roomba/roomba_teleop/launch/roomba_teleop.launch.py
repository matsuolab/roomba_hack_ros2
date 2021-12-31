
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            node_executable='joy_node',
            output='screen'),
        Node(
            package='roomba_teleop',
            node_executable='roomba_teleop',
            output='screen'),
    ])