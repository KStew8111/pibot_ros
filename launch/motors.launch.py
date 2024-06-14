#
# Launch NVIDIA JetBot motor controller and camera nodes.
# This is for the original NVIDIA JetBot.
#
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    motor_controller = Node(package='jetbot_ros', node_executable='motors_pi',
                            output='screen', emulate_tty=True)

    # oled_controller = Node(package='jetbot_ros', node_executable='oled_ssd1306',
    #                        output='screen', emulate_tty=True)

    return LaunchDescription([
        motor_controller,
        # oled_controller,
    ])

