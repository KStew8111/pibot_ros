import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir,LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

from ament_index_python.packages import get_package_share_directory
 
def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    world_file_name = 'maze.world'
    pkg_dir = get_package_share_directory('jetbot_ros')
 
    os.environ["GAZEBO_MODEL_PATH"] = os.path.join(pkg_dir, 'models')
 
    world = os.path.join(pkg_dir, 'worlds', world_file_name)
    launch_file_dir = os.path.join(pkg_dir, 'launch')

    robot_x = DeclareLaunchArgument('x', default_value='0.0')
    robot_y = DeclareLaunchArgument('y', default_value='0.0')
    robot_z = DeclareLaunchArgument('z', default_value='0.0')

    gz_bridge_config = os.path.join(get_package_share_directory('jetbot_ros'), 'config', 'bridge.yaml')
    nav2_params = os.path.join(get_package_share_directory('jetbot_ros'), 'config', 'nav2_params.yaml')
    slam_params = os.path.join(get_package_share_directory('jetbot_ros'), 'config', 'slam_params.yaml')
 
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch/'),
            'gz_sim.launch.py'
        ]),
        launch_arguments=[
            ('gz_args', str(world + ' -v 4 -r'))
        ]
    )

    spawn_jetbot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_jetbot',
        arguments=[
            '-world', 'default',
            '-file', str(os.path.join(pkg_dir, 'models/', 'jetbot/', 'model.sdf')),
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
        ]
    )

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        parameters=[{
            "config_file": gz_bridge_config
        }],
    )

    slam_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('nav2_bringup'), 'launch/'),
            'slam_launch.py'
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params,
            'slam_params_file': slam_params
        }.items()
    )

    localization_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('nav2_bringup'), 'launch/'),
            'navigation_launch.py'
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params,
        }.items()
    )
                     
    return LaunchDescription([
        # robot_x,
        # robot_y,
        # robot_z,
        # gazebo,
        # spawn_jetbot,
        # localization_node,
        slam_node,
    ])
