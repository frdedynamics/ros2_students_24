import launch
import launch_ros
import os
import xacro

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # package_name = 'braitenberg_vehicle'
    # xacro_file_path = os.path.join(get_package_share_directory(package_name), 'urdf', 'turtlebot3_waffle_pi.urdf.xacro')
    # robot_description = xacro.process_file(xacro_file_path).toxml()

    package_name_robot = 'multi_robot_challenge_23'
    xacro_file_name = 'turtlebot3_waffle_pi.urdf.xacro'
    xacro_file_path = os.path.join(get_package_share_directory(package_name_robot), 'urdf', xacro_file_name)
    mesh_dir = os.path.join(get_package_share_directory('turtlebot3_description'),'meshes')

    tb3_0_robot_description = xacro.process_file(
        xacro_file_path, 
        mappings={
            "mesh_dir": mesh_dir,
            "namespace": 'tb3_0',
        },
    ).toxml()

    tb3_1_robot_description = xacro.process_file(
        xacro_file_path, 
        mappings={
            "mesh_dir": mesh_dir,
            "namespace": 'tb3_1',
        },
    ).toxml()


   

    return launch.LaunchDescription([
        
    ])
