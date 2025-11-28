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

    default_world = os.path.join(
        get_package_share_directory('rc_quadruped_ros2_simulation_bringup'),
        'resources', 'worlds', world_file + '.world'
    )

    pkg_path = os.path.join(get_package_share_directory(pkg_description))
    xacro_file = os.path.join(pkg_path, 'xacro', 'robot.xacro')

    robot_description = xacro.process_file(xacro_file, mappings={'GAZEBO': 'true', 'CLASSIC': 'true'}).toxml()

    rviz_config_file = os.path.join(
        pkg_path, "config", "visualize_urdf.rviz"
    )

    # --- RVIZ ---
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=["-d", rviz_config_file]
    )

    # ---- Classic Gazebo ----
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])
        ]),
        launch_arguments={'world': default_world,'verbose': 'true'}.items()
    )

    # --- Spawn robot ---
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'robot',
            '-x', '0', '-y', '0', '-z', init_height,
            '-Y', '1.57'
        ],
        output='screen'
    )
    
    # --- Robot State Publisher ---
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {
                'publish_frequency': 20.0,
                'use_tf_static': True,
                'robot_description': robot_description,
                'ignore_timestamp': True
            }
        ],
    )

    # --- Controllers ---
    joint_state_publisher = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager"],
    )

    imu_sensor_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["imu_sensor_broadcaster",
                   "--controller-manager", "/controller_manager"],
    )
    
    leg_pd_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["leg_pd_controller",
                   "--controller-manager", "/controller_manager"],
    )

    unitree_guide_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["unitree_guide_controller", "--controller-manager", "/controller_manager"],
    )

    return [
        rviz,
        gazebo,
        robot_state_publisher,
        spawn_entity,
        leg_pd_controller,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=leg_pd_controller,
                on_exit=[imu_sensor_broadcaster, joint_state_publisher, unitree_guide_controller],
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
