import os

from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pibot_urdf = os.path.join(get_package_share_directory(
        'pibot_ros'), 'urdf', 'pibot.urdf.xacro')
    rviz_config_file = os.path.join(
        get_package_share_directory('pibot_ros'), 'rviz', 'pibot.rviz')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['xacro ', str(pibot_urdf)]), value_type=str
            ),
        }],
        arguments={

        }
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        # parameters=[{
        #     'use_sim_time': use_sim_time
        # }]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher,
        rviz
    ])
