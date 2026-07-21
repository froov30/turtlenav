# Copyright 2026 Dhruv
# SPDX-License-Identifier: MIT

"""Launch the autonomous chaser configuration without manual or PID control."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Create the autonomous TurtleNav launch description."""
    return LaunchDescription([
        Node(package='turtlesim', executable='turtlesim_node',
             name='turtlesim', output='screen'),
        Node(package='turtle_game', executable='turtle_spawner',
             name='turtle_spawner', output='screen'),
        Node(package='turtle_game', executable='game_manager',
             name='game_manager', output='screen',
             parameters=[{'auto_start': True}]),
        Node(package='turtle_game', executable='target_chaser',
             name='target_chaser', output='screen'),
        Node(package='turtle_game', executable='kalman_filter_node',
             name='kalman_filter_node', output='screen'),
        Node(package='turtle_game', executable='occupancy_grid_mapper',
             name='occupancy_grid_mapper', output='screen'),
        Node(package='turtle_game', executable='watchdog',
             name='watchdog', output='screen'),
        Node(package='turtle_game', executable='data_logger',
             name='data_logger', output='screen'),
    ])
