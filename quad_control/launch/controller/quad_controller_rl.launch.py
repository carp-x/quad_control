import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():

    pkg_file = get_package_share_directory('quad_control')

    urdf_file = os.path.join(pkg_file, 'model', 'anymal_c', 'urdf', 'anymal.urdf')
    task_file = os.path.join(pkg_file, 'config', 'mpc', 'task.info')
    reference_file = os.path.join(pkg_file, 'config', 'command', 'reference.info')
    policy_file = os.path.join(pkg_file, 'policy', 'anymal-c.onnx')

    declare_rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true', description='Whether to start RViz2')
    rviz_config_file = os.path.join(
        get_package_share_directory('quad_control'), 'rviz', 'quad_robot.rviz')

    config_path = os.path.join(
        get_package_share_directory('quad_control'),
        'config',
        'controller',
        'quad_controller_rl.yaml'
    )

    quad_control_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
        "quad_controller_rl",
        "--service-call-timeout", "60",
        "--switch-timeout", "60",
        "--param-file", config_path,
        f"--controller-ros-args=-p task_file:={task_file} "
        f"-p urdf_file:={urdf_file} "
        f"-p reference_file:={reference_file} "
        f"-p policy_file:={policy_file}",
        ],
        parameters=[{
        'use_sim_time': True,
        }],
    )

    quad_control_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='quad_control_rviz',
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d', rviz_config_file],
        parameters=[{
            'use_sim_time': True,
        }],
    )

    return LaunchDescription([
        quad_control_spawner,
        declare_rviz_arg,
        quad_control_rviz,
    ])
