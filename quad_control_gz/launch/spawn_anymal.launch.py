import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    pkg_project = get_package_share_directory('quad_control_gz')

    urdf_file = os.path.join(pkg_project, 'urdf', 'anymal.complete.xacro')
    robot_description_config = xacro.process_file(urdf_file)
    robot_desc = robot_description_config.toxml()

    pkg_meshes = get_package_share_directory('ocs2_robotic_assets')
    mesh_file = os.path.dirname(pkg_meshes)
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[mesh_file + ':' + os.path.join(pkg_project, 'share')]
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True
        }],
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'anymal',
            '-z', '0.6'
        ],
        parameters=[{'use_sim_time': True}],
    )

    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        parameters=[{'use_sim_time': True}],
    )

    imu_sensor_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["imu_sensor_broadcaster"],
        parameters=[{'use_sim_time': True}],
    )

    # lf_ft_sensor_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["lf_ft_sensor_broadcaster"],
    #     parameters=[{'use_sim_time': True}],
    # )

    # rf_ft_sensor_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["rf_ft_sensor_broadcaster"],
    #     parameters=[{'use_sim_time': True}],
    # )

    # lh_ft_sensor_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["lh_ft_sensor_broadcaster"],
    #     parameters=[{'use_sim_time': True}],
    # )

    # rh_ft_sensor_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["rh_ft_sensor_broadcaster"],
    #     parameters=[{'use_sim_time': True}],
    # )

    bridge_clock = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    return LaunchDescription([
        set_gz_resource_path,        # 先设路径
        node_robot_state_publisher,  # 发布模型
        gz_sim,                      # 开仿真器
        gz_spawn_entity,             # 生成实体
        joint_state_spawner,
        imu_sensor_spawner,
        # lf_ft_sensor_spawner,
        # rf_ft_sensor_spawner,
        # lh_ft_sensor_spawner,
        # rh_ft_sensor_spawner,
        bridge_clock,
    ])
