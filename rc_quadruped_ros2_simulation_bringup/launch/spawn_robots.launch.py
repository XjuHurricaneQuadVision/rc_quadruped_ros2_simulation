import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def launch_setup(context, *args, **kwargs):
    pkg_description = context.launch_configurations['pkg_description']
    init_height = context.launch_configurations['height']
    world_file = context.launch_configurations['world_file']

    pkg_path = get_package_share_directory(pkg_description)
    xacro_file = os.path.join(pkg_path, 'xacro', 'robot.xacro')

    # -------------------------------
    # 🔑 Ignition / ros_gz_sim 资源路径（关键修复）
    # -------------------------------
    models_path = os.path.join(pkg_path, 'models')

    os.environ['IGN_GAZEBO_RESOURCE_PATH'] = (
        os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '') + ':' + models_path
    )
    os.environ['GZ_SIM_RESOURCE_PATH'] = (
        os.environ.get('GZ_SIM_RESOURCE_PATH', '') + ':' + models_path
    )

    # -------------------------------
    # robot_description
    # -------------------------------
    robot_description = xacro.process_file(
        xacro_file,
        mappings={
            'GAZEBO': 'true',
            'EXTERNAL_SENSORS': 'true'
        }
    ).toxml()

    rviz_config_file = os.path.join(pkg_path, "config", "visualize_urdf.rviz")

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=["-d", rviz_config_file]
    )

    # -------------------------------
    # World & models
    # -------------------------------
    world_path = os.path.join(
        get_package_share_directory('rc_quadruped_ros2_simulation_bringup'),
        'resources', 'worlds', world_file + '.sdf'
    )

    mid360_sdf = os.path.join(models_path, 'mid360', 'model.sdf')

    # -------------------------------
    # Robot State Publisher
    # -------------------------------
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'publish_frequency': 50.0,
            'use_tf_static': True
        }]
    )

    # -------------------------------
    # Spawn robot & sensors
    # -------------------------------
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'robot',
            '-allow_renaming', 'true',
            '-z', init_height
        ],
    )

    gz_spawn_mid360 = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-file', mid360_sdf,
            '-name', 'mid360',
            '-attach', 'livox_frame',
        ]
    )

    # -------------------------------
    # Controllers
    # -------------------------------
    joint_state_publisher = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    imu_sensor_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["imu_sensor_broadcaster", "--controller-manager", "/controller_manager"],
    )

    leg_pd_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["leg_pd_controller", "--controller-manager", "/controller_manager"],
    )

    unitree_guide_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["unitree_guide_controller", "--controller-manager", "/controller_manager"],
    )

    return [
        rviz,
        robot_state_publisher,
        gz_spawn_entity,
        gz_spawn_mid360,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('ros_gz_sim'),
                    'launch',
                    'gz_sim.launch.py'
                ])
            ),
            launch_arguments=[
                ('gz_args', [' -r -v 4 ', world_path])
            ]
        ),
        leg_pd_controller,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=leg_pd_controller,
                on_exit=[
                    imu_sensor_broadcaster,
                    joint_state_publisher,
                    unitree_guide_controller
                ],
            )
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('pkg_description', default_value='go2_description'),
        DeclareLaunchArgument('height', default_value='0.5'),
        DeclareLaunchArgument('world_file', default_value='default'),
        OpaqueFunction(function=launch_setup)
    ])
