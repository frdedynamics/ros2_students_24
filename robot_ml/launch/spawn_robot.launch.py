import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # Get the path to the Gazebo world file
    package_name = 'robot_ml'  # Replace with your package name
    world_file_path = os.path.join(get_package_share_directory(package_name), 'worlds', 'ml_exercise.world')
    # Get the path to model folder
    model_path = os.path.join(get_package_share_directory(package_name), 'models')
    # Get the path to the Xacro file
    package_name_robot = 'braitenberg_vehicle'
    xacro_file_path = os.path.join(get_package_share_directory(package_name_robot), 'urdf', 'turtlebot3_waffle_pi.urdf.xacro')

    os.environ['GAZEBO_MODEL_PATH'] = model_path

    tb3_pos = ['0.0', '0.0', '0.0']
    tb3_yaw = '1.57079632679'

    # Use xacro to process the file
    robot_description_raw = xacro.process_file(xacro_file_path).toxml()

    # Declaring use_sim_time as a launch argument that can then be used in all launch files
    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    # Get launch argument use_sim_time as a launch configuration object
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Starting Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        launch_arguments={'world': world_file_path}.items()
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_raw,
            'use_sim_time': use_sim_time
        }]
    )

    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'tb3',
            '-x', tb3_pos[0], '-y', tb3_pos[1], '-z', tb3_pos[2], '-Y', tb3_yaw
        ],
        output='screen'
    )

    # Run the node
    return LaunchDescription([
        sim_time_arg,
        gazebo,
        node_robot_state_publisher,
        spawn_entity
    ])
