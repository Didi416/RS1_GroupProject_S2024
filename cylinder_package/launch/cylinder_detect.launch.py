import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_cylinder = get_package_share_directory('cylinder_package')
    pkg_turtlebot3 = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_turtlebot_nav = get_package_share_directory('turtlebot3_navigation2')

    world = os.path.join(pkg_cylinder, 'world', 'cylinder_world.world')
    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    rviz_config_dir = os.path.join(pkg_cylinder, 'rviz', 'cylinder_nav.rviz')

    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(pkg_cylinder, 'maps', 'map3.yaml'))

    param_file_name = 'waffle_pi.yaml'
    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(pkg_turtlebot_nav, 'param', param_file_name))
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true'),
    x_pose = LaunchConfiguration('x_pose', default='-3.0'),
    y_pose = LaunchConfiguration('y_pose', default='2.0'),
    
    gazebo_LD = LaunchDescription([
        # Nav2 Launch directives
        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'params_file',
            default_value=param_dir,
            description='Full path to param file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_launch_file_dir, 'bringup_launch.py')
            ),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': param_dir
            }.items(),
        ),

        # Gazebo Launch directives
        DeclareLaunchArgument(
            'model',
            default_value='waffle_pi',
            description="TurtleBot3 model"
        ),
        SetEnvironmentVariable(
            'TURTLEBOT3_MODEL', LaunchConfiguration('model')),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world}.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            )
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_turtlebot3, 'robot_state_publisher.launch.py')
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_turtlebot3, 'spawn_turtlebot3.launch.py')
            ),
            launch_arguments={
                'x_pose': x_pose,
                'y_pose': y_pose
            }.items()
        ),

        # Launch RVIZ Node
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
        
        # Launch executable nodes
        Node( # Navigation node
            package='cylinder_package',
            executable='initial_pose',
            name='initial_pose',
            output='screen',
        ),

        Node( # Navigation node
            package='cylinder_package',
            executable='cylinder_rotate',
            name='cylinder_detect',
            output='screen',
        )
    ])

    return gazebo_LD