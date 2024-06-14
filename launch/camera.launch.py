#
# Launch NVIDIA JetBot motor controller and camera nodes.
# This is for the original NVIDIA JetBot.
#
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
# from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch_ros.actions import Node


def generate_launch_description():

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('depthai_ros_driver'), 'launch/'),
            'pointcloud.launch.py'
        ])
    )

    # oled_controller = Node(package='jetbot_ros', node_executable='oled_ssd1306',
    #                        output='screen', emulate_tty=True)

    return LaunchDescription([
        camera,
        # oled_controller,
    ])

