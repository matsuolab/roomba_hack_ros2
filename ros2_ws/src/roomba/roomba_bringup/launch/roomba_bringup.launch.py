import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    roomba_description_launch_dir = os.path.join(get_package_share_directory('roomba_description'), 'launch') 
    roomba_description_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([roomba_description_launch_dir, '/display.launch.py'])
    )  

    ca_driver_cmd = Node(
        package="create_driver",
        node_executable="create_driver",
        output="screen",
        parameters=[{'dev': '/dev/roomba'},
                    {'latch_cmd_duration': 0.2},
                    {'loop_hz': 10.0},
                    {'publish_tf': True}]
    )

    rplidar_cmd = Node(
        package='rplidar_ros',
        node_executable='rplidar_node',
        output='screen',
        parameters=[{'channel_type': "serial"},
                    {'serial_port': "/dev/rplidar"},
                    {'serial_buadrate': 115200},
                    {'frame_id': "rplidar_laser_link"}]
    )

    ld = LaunchDescription()
    ld.add_action(roomba_description_cmd)
    ld.add_action(ca_driver_cmd)
    ld.add_action(rplidar_cmd)

    return ld