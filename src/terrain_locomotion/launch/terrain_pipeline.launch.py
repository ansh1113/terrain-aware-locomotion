"""Full terrain perception and planning pipeline launch file.

Launches all nodes needed for terrain-aware locomotion without simulation.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for terrain pipeline."""
    
    pkg_terrain_locomotion = FindPackageShare('terrain_locomotion')
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true'
    )
    
    enable_safety_arg = DeclareLaunchArgument(
        'enable_safety',
        default_value='true',
        description='Enable safety monitor'
    )
    
    # Perception nodes
    terrain_classifier = Node(
        package='terrain_locomotion',
        executable='terrain_classifier',
        name='terrain_classifier',
        output='screen',
        parameters=[
            PathJoinSubstitution([pkg_terrain_locomotion, 'config', 'terrain_classifier.yaml']),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    elevation_mapper = Node(
        package='terrain_locomotion',
        executable='elevation_mapper',
        name='elevation_mapper',
        output='screen',
        parameters=[
            PathJoinSubstitution([pkg_terrain_locomotion, 'config', 'elevation_mapper.yaml']),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    # Planning node
    footstep_planner = Node(
        package='terrain_locomotion',
        executable='footstep_planner',
        name='footstep_planner',
        output='screen',
        parameters=[
            PathJoinSubstitution([pkg_terrain_locomotion, 'config', 'footstep_planner.yaml']),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    # Control node
    gait_controller = Node(
        package='terrain_locomotion',
        executable='gait_controller',
        name='gait_controller',
        output='screen',
        parameters=[
            PathJoinSubstitution([pkg_terrain_locomotion, 'config', 'gait_controller.yaml']),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    # Safety monitor
    safety_monitor = Node(
        package='terrain_locomotion',
        executable='safety_monitor',
        name='safety_monitor',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'enable_safety_checks': LaunchConfiguration('enable_safety')
        }]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        enable_safety_arg,
        terrain_classifier,
        elevation_mapper,
        footstep_planner,
        gait_controller,
        safety_monitor
    ])
