# File: ~/terrain_locomotion_ws/src/terrain_locomotion/launch/perception_only.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true'
    )
    
    # Terrain classifier only
    terrain_classifier = Node(
        package='terrain_locomotion',
        executable='terrain_classifier',
        name='terrain_classifier',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'confidence_threshold': 0.8,
            'model_path': ''  # Empty for demo mode
        }]
    )
    
    # Elevation mapper only
    elevation_mapper = Node(
        package='terrain_locomotion',
        executable='elevation_mapper',
        name='elevation_mapper',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'map_size': 200,
            'resolution': 0.05,
            'max_range': 5.0,
            'min_range': 0.1
        }]
    )
    
    # RViz2 for visualization
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('terrain_locomotion'),
            'config',
            'perception_demo.rviz'
        ])],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        terrain_classifier,
        elevation_mapper,
        rviz2
    ])