# File: ~/terrain_locomotion_ws/src/terrain_locomotion/launch/terrain_demo.launch.py

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Find the share directory for the packages
    pkg_terrain_locomotion = FindPackageShare('terrain_locomotion')
    pkg_terrain_description = FindPackageShare('terrain_description')
    pkg_gazebo_ros = FindPackageShare('gazebo_ros')

    # --- LAUNCH ARGUMENTS ---
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # --- GAZEBO LAUNCH ---
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                pkg_gazebo_ros,
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': PathJoinSubstitution([pkg_terrain_locomotion, 'worlds', 'terrain_world.world'])
        }.items()
    )
    
    # --- ROBOT DESCRIPTION ---
    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([pkg_terrain_description, 'urdf', 'anymal_simple.urdf.xacro'])
    ])

    # --- NODES ---
    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # Spawner: Spawn the robot entity in Gazebo at the world's starting area
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'anymal_simple',
                   '-x', '2.0',
                   '-y', '0.0',
                   '-z', '0.5'],
        output='screen'
    )

    # Spawner: Load the Joint State Broadcaster
    spawn_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Spawner: Load the Joint Trajectory Controller
    spawn_joint_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Your application nodes
    terrain_classifier = Node(package='terrain_locomotion', executable='terrain_classifier', name='terrain_classifier', output='screen')
    elevation_mapper = Node(package='terrain_locomotion', executable='elevation_mapper', name='elevation_mapper', output='screen')
    footstep_planner = Node(package='terrain_locomotion', executable='footstep_planner', name='footstep_planner', output='screen')
    gait_controller = Node(package='terrain_locomotion', executable='gait_controller', name='gait_controller', output='screen')
    demo_pipeline = Node(package='terrain_locomotion', executable='demo_pipeline', name='demo_pipeline', output='screen')
    
    # RViz
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([pkg_terrain_locomotion, 'config', 'terrain_demo.rviz'])]
    )
    
    # --- LAUNCH DESCRIPTION ---
    return LaunchDescription([
        use_sim_time_arg,
        gazebo_launch,
        robot_state_publisher_node,
        spawn_entity_node,

        # Use an event handler to chain the controller spawners.
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity_node,
                on_exit=[spawn_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_joint_state_broadcaster,
                on_exit=[spawn_joint_trajectory_controller],
            )
        ),
        
        # Your application nodes
        terrain_classifier,
        elevation_mapper,
        footstep_planner,
        gait_controller,
        demo_pipeline,
        rviz2_node
    ])