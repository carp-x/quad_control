import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():

  pkg_robot = get_package_share_directory('quad_control')
  pkg_mpc = get_package_share_directory('quad_control')
  pkg_wbc = get_package_share_directory('quad_control')

  urdf_file = os.path.join(pkg_robot, 'model', 'anymal_c', 'urdf', 'anymal.urdf')
  task_file = os.path.join(pkg_mpc, 'config', 'mpc', 'task.info')
  reference_file = os.path.join(pkg_mpc, 'config', 'command', 'reference.info')
  task_file_wbc = os.path.join(pkg_wbc, 'config', 'wbc', 'task.info')

  declare_rviz_arg = DeclareLaunchArgument(
      'rviz', default_value='true', description='Whether to start RViz2')
  rviz_config_file = os.path.join(
      get_package_share_directory('quad_control'), 'rviz', 'quad_robot.rviz')

  config_path = os.path.join(
    get_package_share_directory('quad_control'),
    'config',
    'controller',
    'quad_controller.yaml'
  )

  quad_controller_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=[
      "quad_controller",
      "--service-call-timeout", "60",
      "--switch-timeout", "60",
      "--param-file", config_path,
      f"--controller-ros-args=-p task_file:={task_file} -p urdf_file:={urdf_file} -p reference_file:={reference_file}",
      f"--controller-ros-args=-p task_file_wbc:={task_file_wbc}",
    ],
    parameters=[{
      'use_sim_time': True,
    }],
  )

  quad_controller_rviz = Node(
    package='rviz2',
    executable='rviz2',
    name='quad_controller_rviz',
    output='screen',
    condition=IfCondition(LaunchConfiguration('rviz')),
    arguments=['-d', rviz_config_file],
    parameters=[{
        'use_sim_time': True,
    }],
  )

  return LaunchDescription([
    quad_controller_spawner,
    declare_rviz_arg,
    quad_controller_rviz,
  ])
