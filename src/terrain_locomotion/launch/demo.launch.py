"""Complete demo launch file.

Launches simulation and full terrain-aware locomotion pipeline.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """Generate launch description for complete demo."""
    
    pkg_terrain_locomotion = FindPackageShare('terrain_locomotion')
    
    # Launch arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='terrain_world.world',
        description='World file to load (terrain_world.world or flat_world.world)'
    )
    
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Start Gazebo with GUI'
    )
    
    demo_mode_arg = DeclareLaunchArgument(
        'demo_mode',
        default_value='true',
        description='Use demo mode with feature-based classification'
    )
    
    # Include simulation launch
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                pkg_terrain_locomotion,
                'launch',
                'simulation.launch.py'
            ])
        ]),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'gui': LaunchConfiguration('gui')
        }.items()
    )
    
    # Include terrain pipeline launch
    pipeline_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                pkg_terrain_locomotion,
                'launch',
                'terrain_pipeline.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            'enable_safety': 'true'
        }.items()
    )
    
    # Demo pipeline node (sends velocity commands)
    demo_pipeline = Node(
        package='terrain_locomotion',
        executable='demo_pipeline',
        name='demo_pipeline',
        output='screen',
        parameters=[{
            'use_sim_time': True
        }]
    )
    
    # RViz for visualization (optional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[
            '-d', PathJoinSubstitution([
                pkg_terrain_locomotion,
                'config',
                'terrain_demo.rviz'
            ])
        ],
        parameters=[{'use_sim_time': True}],
        condition=LaunchConfiguration('gui')  # Only launch if GUI is enabled
    )
    
    return LaunchDescription([
        world_arg,
        gui_arg,
        demo_mode_arg,
        simulation_launch,
        pipeline_launch,
        demo_pipeline,
        # rviz_node  # Uncomment when RViz config is available
    ])
