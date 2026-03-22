import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    rviz_config_file = get_package_share_directory('quad_control') + "/rviz/legged_robot.rviz"
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='terminal_prefix',
            default_value=''
        ),
        launch.actions.DeclareLaunchArgument(
            name='referenceFile',
            default_value=get_package_share_directory(
                'quad_control') + '/config/command/reference.info'
        ),
        launch.actions.DeclareLaunchArgument(
            name='gaitCommandFile',
            default_value=get_package_share_directory(
                'quad_control') + '/config/command/gait.info'
        ),

        launch_ros.actions.Node(
            package='quad_control_ros',
            executable='legged_robot_target',
            name='quad_robot_target',
            output='screen',
            prefix=launch.substitutions.LaunchConfiguration('terminal_prefix'),
            parameters=[
                {
                    'referenceFile': launch.substitutions.LaunchConfiguration('referenceFile')
                }
            ]
        ),
        launch_ros.actions.Node(
            package='quad_control_ros',
            executable='legged_robot_gait_command',
            name='quad_robot_gait_command',
            output='screen',
            prefix=launch.substitutions.LaunchConfiguration('terminal_prefix'),
            parameters=[
                {
                    'gaitCommandFile': launch.substitutions.LaunchConfiguration('gaitCommandFile')
                }
            ]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
