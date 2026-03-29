import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    pkg_dir = get_package_share_directory('quad_control')
    
    launch_command = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch','command' , 'quad_robot_command.launch.py')
        ),
    )

    launch_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'gazebo', 'gazebo_sim.launch.py')
        ),
    )

    launch_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'controller', 'quad_controller.launch.py')
        ),
    )

    return LaunchDescription([
        launch_command,
        launch_gazebo,
        launch_controller,
    ])
