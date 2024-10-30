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
    # get directory paths to necessary packages:
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_rs1_project = get_package_share_directory('rs1_group_project')
    pkg_turtlebot3 = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_turtlebot_nav = get_package_share_directory('turtlebot3_navigation2')

    # identify paths to requried world, nav2 configuration, rviz configuration files as well as the map and param .yaml files
    world = os.path.join(pkg_rs1_project, 'worlds', 'rs1_project_world.world')
    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    rviz_config_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'rviz', 'nav2_default_view.rviz')
    
    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(pkg_rs1_project, 'maps', 'map3.yaml'))

    param_file_name = 'waffle_pi.yaml'
    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(pkg_turtlebot_nav, 'param', param_file_name))
    
    # set use_sim_time as well as global coordinates for the turtlebot upon launch
    use_sim_time = LaunchConfiguration('use_sim_time', default='true'),
    x_pose = LaunchConfiguration('x_pose', default='-2.0'),
    y_pose = LaunchConfiguration('y_pose', default='2.0'),

    # Start Launch Description
    gazebo_LD = LaunchDescription([
        # Nav2 Launch directives/arguments
        # declare/setup arguments for map, param, use_sim_time
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
        # Include previous launch arguments in Nav2 stack launch description (arguments will be input into the bringup_launch file)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_launch_file_dir, 'bringup_launch.py')
            ),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': param_dir,
            }.items(),
        ),

        # Gazebo Launch directives
        # Set turtlebot model as waffle_pi
        DeclareLaunchArgument(
            'model',
            default_value='waffle_pi',
            description="TurtleBot3 model"
        ),
        # Set environmental variable in launch file so don't have to do so in terminal every time
        SetEnvironmentVariable(
            'TURTLEBOT3_MODEL', LaunchConfiguration('model')),
        # Include in launch description the gazebo server and client files as well as the robot_state publisher and spawn turtlebot files
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
            # Set turtlebot to specified global coordinates
            launch_arguments={
                'x_pose': x_pose,
                'y_pose': y_pose,
            }.items()
        ),

        # Launch RVIZ Node with rviz configuration file as argument (not the default file, but .rviz we have in this directory)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
        
        # Launch executable nodes
        Node( # initial_pose node to publish the robot pose to rviz/nav2 stack to localise and ensure ready to assume navigation upon startup, no manual input required
            package='rs1_group_project',
            executable='initial_pose',
            name='initial_pose_set',
            output='screen',
        ),

        # Node( # object detection node
        #     package='rs1_group_project',
        #     executable='detect',
        #     name='object_detection',
        #     output='screen',
        # ),

        # Node( # manual control node
        #     package='rs1_group_project',
        #     executable='userInput',
        #     name='user_input',
        #     output='screen',
        # ),
    ])
    # return the LaunchDescription initialised from line 39 which includes all launch arguments/configurations/descriptions from line 39-140
    return gazebo_LD
