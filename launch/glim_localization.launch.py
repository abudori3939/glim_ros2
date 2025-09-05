#!/usr/bin/env python3

import os
import launch
import launch.conditions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package share directory
    glim_share_dir = get_package_share_directory('glim')
    
    # Declare launch arguments
    config_path_arg = DeclareLaunchArgument(
        'config_path',
        default_value=os.path.join(glim_share_dir, 'config'),
        description='Path to configuration directory'
    )
    
    map_path_arg = DeclareLaunchArgument(
        'map_path',
        default_value='/home/yuta/data/glim/kasukabe',
        #default_value='/tmp/dump',
        description='Path to the saved map (dump directory)'
    )
    
    initial_x_arg = DeclareLaunchArgument(
        'initial_x',
        default_value='0.0',
        description='Initial X position in map coordinates'
    )
    
    initial_y_arg = DeclareLaunchArgument(
        'initial_y',
        default_value='0.0',
        description='Initial Y position in map coordinates'
    )
    
    initial_z_arg = DeclareLaunchArgument(
        'initial_z',
        default_value='0.0',
        description='Initial Z position in map coordinates'
    )
    
    initial_roll_arg = DeclareLaunchArgument(
        'initial_roll',
        default_value='0.0',
        description='Initial roll angle in radians'
    )
    
    initial_pitch_arg = DeclareLaunchArgument(
        'initial_pitch',
        default_value='0.0',
        description='Initial pitch angle in radians'
    )
    
    initial_yaw_arg = DeclareLaunchArgument(
        'initial_yaw',
        default_value='0.0',
        description='Initial yaw angle in radians'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz for visualization'
    )
    
    dump_on_shutdown_arg = DeclareLaunchArgument(
        'dump_on_shutdown',
        default_value='false',
        description='Save map on shutdown'
    )
    
    # GLIM Localization Node
    # Note: Topics (imu_topic, points_topic) are configured in config_ros.json
    glim_localization_node = Node(
        package='glim_ros',
        executable='glim_localization_node',
        name='glim_localization',
        output='screen',
        parameters=[{
            'config_path': LaunchConfiguration('config_path'),
            'map_path': LaunchConfiguration('map_path'),
            'initial_pose.x': LaunchConfiguration('initial_x'),
            'initial_pose.y': LaunchConfiguration('initial_y'),
            'initial_pose.z': LaunchConfiguration('initial_z'),
            'initial_pose.roll': LaunchConfiguration('initial_roll'),
            'initial_pose.pitch': LaunchConfiguration('initial_pitch'),
            'initial_pose.yaw': LaunchConfiguration('initial_yaw'),
            'dump_on_shutdown': LaunchConfiguration('dump_on_shutdown'),
        }]
    )
    
    # RViz Node (optional)
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('glim_ros'),
        'rviz',
        'glim_ros.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        condition=launch.conditions.IfCondition(LaunchConfiguration('use_rviz'))
    )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(config_path_arg)
    ld.add_action(map_path_arg)
    ld.add_action(initial_x_arg)
    ld.add_action(initial_y_arg)
    ld.add_action(initial_z_arg)
    ld.add_action(initial_roll_arg)
    ld.add_action(initial_pitch_arg)
    ld.add_action(initial_yaw_arg)
    ld.add_action(use_rviz_arg)
    ld.add_action(dump_on_shutdown_arg)
    
    # Add nodes
    ld.add_action(glim_localization_node)
    
    # Add RViz if requested
    if True:  # Always include RViz node definition, condition will be checked at runtime
        ld.add_action(rviz_node)
    
    return ld
