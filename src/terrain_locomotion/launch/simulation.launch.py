"""Simulation launch file for terrain-aware locomotion.

Launches Gazebo simulation with robot and controllers.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """Generate launch description for simulation."""
    
    # Find package shares
    pkg_terrain_locomotion = FindPackageShare('terrain_locomotion')
    pkg_terrain_description = FindPackageShare('terrain_description')
    pkg_gazebo_ros = FindPackageShare('gazebo_ros')
    
    # Launch arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='comprehensive_terrain.world',
        description='World file to load'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Start Gazebo with GUI'
    )
    
    # Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                pkg_gazebo_ros,
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': PathJoinSubstitution([
                pkg_terrain_locomotion,
                'worlds',
                LaunchConfiguration('world')
            ]),
            'gui': LaunchConfiguration('gui'),
            'verbose': 'false',
            'pause': 'false'
        }.items()
    )
    
    # Robot description
    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([
            pkg_terrain_description,
            'urdf',
            'anymal_simple.urdf.xacro'
        ])
    ])
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    # Spawn robot in Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_robot',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'anymal',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.7',  # Spawn higher to avoid ground collision on spawn
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0'
        ]
    )
    
    # Joint state broadcaster spawner
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        name='joint_state_broadcaster_spawner',
        output='screen',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager'
        ]
    )
    
    # Joint trajectory controller spawner
    joint_trajectory_controller = Node(
        package='controller_manager',
        executable='spawner',
        name='joint_trajectory_controller_spawner',
        output='screen',
        arguments=[
            'joint_trajectory_controller',
            '--controller-manager',
            '/controller_manager'
        ]
    )
    
    # Simple walking demo node (started after controllers are ready)
    walking_demo = Node(
        package='terrain_locomotion',
        executable='simple_walk_demo',
        name='simple_walk_demo',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    # Chain actions: spawn robot → load joint state broadcaster → load trajectory controller → start walking demo
    load_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster]
        )
    )
    
    load_joint_trajectory_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[joint_trajectory_controller]
        )
    )
    
    # Start walking demo after a delay to ensure controllers are ready
    start_walking_demo = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_trajectory_controller,
            on_exit=[TimerAction(
                period=3.0,  # Wait 3 seconds after controller loads
                actions=[walking_demo]
            )]
        )
    )
    
    return LaunchDescription([
        world_arg,
        use_sim_time_arg,
        gui_arg,
        gazebo_launch,
        robot_state_publisher,
        spawn_robot,
        load_joint_state_broadcaster,
        load_joint_trajectory_controller,
        start_walking_demo
    ])
