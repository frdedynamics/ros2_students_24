
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_dir = LaunchConfiguration(
        'map',
        # default=os.path.join(
        #     get_package_share_directory('turtlebot3_navigation2'),
        #     'map',
        #     'turtlebot3_world.yaml'))
        default=os.path.join(
            get_package_share_directory('multi_robot_challenge_23'),
            'maps',
            'map_dat160_w1.yaml'))

    param_file_name = "waffle_pi" + '.yaml'
    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            get_package_share_directory('turtlebot3_navigation2'),
            'param',
            param_file_name))

    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    rviz_config_dir = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'rviz',
        'nav2_default_view.rviz')
    

    map_file_arg = DeclareLaunchArgument(
        'map_file', default_value=os.path.join(
            get_package_share_directory(
                'multi_robot_challenge_23'), 'maps', 'map_dat160_w1.yaml'),
        description='Full path to map file to load')
    params_file_arg = DeclareLaunchArgument(
        'params_file', default_value=os.path.join(
            get_package_share_directory(
                'turtlebot3_navigation2'), 'param', 'waffle_pi.yaml'),
        description='Full path to param file to load')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='True',
        description='Use simulation (Omniverse Isaac Sim) clock if true')

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_launch_file_dir, 'bringup_launch.py')),
        launch_arguments={
            'map': LaunchConfiguration('map_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('params_file')
        }.items(),
        # condition=IfCondition(LaunchConfiguration('run_nav2'))
    )

    return LaunchDescription([
        # DeclareLaunchArgument(
        #     'map',
        #     default_value=map_dir,
        #     description='Full path to map file to load'),

        # # DeclareLaunchArgument(
        # #     'params_file',
        # #     default_value=param_dir,
        # #     description='Full path to param file to load'),

        # DeclareLaunchArgument(
        #     'use_sim_time',
        #     default_value='false',
        #     description='Use simulation (Gazebo) clock if true'),

        # SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        # Node(
        #     package='nav2_map_server',
        #     executable='map_server',
        #     name='map_server',
        #     output='screen',
        #     parameters=[{"yaml_filename": map_dir}],
        #     # remappings=remappings
        # ),

        # Node(
        #     package='nav2_lifecycle_manager',
        #     executable='lifecycle_manager',
        #     name='lifecycle_manager_localization',
        #     output='screen',
        #     parameters=[{'use_sim_time': use_sim_time},
        #                 {'autostart': True},
        #                 {'node_names': ["map_server"]}]
        # ),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
        #     launch_arguments={
        #         'map': map_dir,
        #         'use_sim_time': use_sim_time,
        #         'params_file': param_dir}.items(),
        # ),

        map_file_arg,
        params_file_arg,
        use_sim_time_arg,
        # run_rviz_arg,
        # run_nav2_arg,
        # rviz_launch,
        nav2_launch,
        # baselink_basefootprint_publisher,
        # baselink_lidar_publisher,
        # occupancy_grid_localizer_container,

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])