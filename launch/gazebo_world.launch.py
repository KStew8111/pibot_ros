import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory
 
def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
     
    robot_name = DeclareLaunchArgument('robot_name', default_value='jetbot')
    robot_model = DeclareLaunchArgument('robot_model', default_value='simple_diff_ros')  # jetbot_ros
    jetbot_urdf = os.path.join(get_package_share_directory('jetbot_ros'), 'urdf', 'jetbot.urdf.xacro')
    
    robot_x = DeclareLaunchArgument('x', default_value='0.0')
    robot_y = DeclareLaunchArgument('y', default_value='0.0')
    robot_z = DeclareLaunchArgument('z', default_value='0.0')
    
    world_file_name = 'maze.world'
    pkg_dir = get_package_share_directory('jetbot_ros')
 
    os.environ["GZ_SIM_RESOURCE_PATH"] = os.path.join(pkg_dir, 'models/:') + os.path.join('/home/kyle/jetbot_ws/src')
    print(os.environ["GZ_SIM_RESOURCE_PATH"])
 
    world = os.path.join(pkg_dir, 'worlds', world_file_name)
    launch_file_dir = os.path.join(pkg_dir, 'launch')

    gz_bridge_config = os.path.join(get_package_share_directory('jetbot_ros'), 'config', 'bridge.yaml')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['xacro ', str(jetbot_urdf)]), value_type=str
            ),
        }],
        arguments={
            'use_sim_time': use_sim_time
        }
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch/'),
            'gz_sim.launch.py'
        ]),
        launch_arguments=[
            ('gz_args', str(world + ' -v 4'))
        ]
    )

    spawn_jetbot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_jetbot',
        arguments=[
            '-world', 'default',
            # '-file', str(os.path.join(pkg_dir, 'models/', 'jetbot/', 'model.sdf')),
            # '-file', str(os.path.join(pkg_dir, 'urdf/', 'jetbot.urdf')),
            '-topic', '/robot_description',
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
 
    return LaunchDescription([
        robot_name,
        robot_model, 
        robot_x,
        robot_y,
        robot_z,
        robot_state_publisher,
        joint_state_publisher,
        gazebo,
        ros_gz_bridge,
        spawn_jetbot,
    ])
