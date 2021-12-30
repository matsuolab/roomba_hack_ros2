import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    share_dir_path = os.path.join(get_package_share_directory('roomba_description'))
    xacro_path = os.path.join(share_dir_path, 'urdf', 'roomba.urdf.xacro')
    urdf_path = os.path.join(share_dir_path, 'urdf', 'roomba.urdf')

    doc = xacro.process_file(xacro_path)

    robot_description = doc.toprettyxml(indent=' ')
    with open(urdf_path, 'w') as f:
        f.write(robot_description)
    
    robot_state_publisher_cmd = Node(
        package="robot_state_publisher",
        node_executable="robot_state_publisher",
        output="screen",
        arguments=[urdf_path]
    )

    joint_state_publisher_cmd = Node(
        package="joint_state_publisher",
        node_executable="joint_state_publisher",
        output="screen",
        arguments=[urdf_path]
    )

    ld = LaunchDescription()
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(joint_state_publisher_cmd)

    return ld