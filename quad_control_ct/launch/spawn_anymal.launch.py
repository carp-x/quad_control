import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

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
      "--controller-manager-timeout", "60",
      "--param-file", config_path
    ],
    parameters=[{'use_sim_time': True}],
  )

  return LaunchDescription([
    quad_controller_spawner
  ])
