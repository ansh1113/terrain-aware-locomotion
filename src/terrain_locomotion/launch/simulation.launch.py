"""Simulation launch file for terrain-aware locomotion.

Launches Gazebo simulation with robot and controllers.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
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
        default_value='terrain_world.world',
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
            'verbose': 'false'
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
            '-x', '2.0',
            '-y', '0.0',
            '-z', '0.5'
        ]
    )
    
    # Joint state broadcaster
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
    
    # Joint trajectory controller
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
    
    # Chain controller spawners after robot spawns
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
    
    return LaunchDescription([
        world_arg,
        use_sim_time_arg,
        gui_arg,
        gazebo_launch,
        robot_state_publisher,
        spawn_robot,
        load_joint_state_broadcaster,
        load_joint_trajectory_controller
    ])
