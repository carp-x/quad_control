import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

  pkg_robot = get_package_share_directory('ocs2_robotic_assets')
  pkg_mpc = get_package_share_directory('quad_control_mpc')

  urdf_file = os.path.join(pkg_robot, 'resources', 'anymal_c', 'urdf', 'anymal.urdf')
  task_file = os.path.join(pkg_mpc, 'config', 'mpc', 'task.info')
  reference_file = os.path.join(pkg_mpc, 'config', 'command', 'reference.info')

  config_path = os.path.join(
    get_package_share_directory('quad_control_ct'),
    'config',
    'config.yaml'
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
    ],
    parameters=[{
      'use_sim_time': True,
    }],
  )

  return LaunchDescription([
    quad_controller_spawner
  ])
