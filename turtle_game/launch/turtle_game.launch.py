# Copyright 2026 Dhruv
# SPDX-License-Identifier: MIT

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim',
            output='screen'
        ),
        Node(
            package='turtle_game',
            executable='turtle_spawner',
            name='turtle_spawner',
            output='screen'
        ),
        Node(
            package='turtle_game',
            executable='game_manager',
            name='game_manager',
            output='screen',
            prefix='xterm -e'
        ),
        Node(
            package='turtle_game',
            executable='player_controller',
            name='player_controller',
            output='screen',
            prefix='xterm -e'
        ),
        Node(
            package='turtle_game',
            executable='pid_controller',
            name='pid_controller',
            output='screen',
            prefix='xterm -e',
            parameters=[{'enabled': False}]
        ),
        Node(
            package='turtle_game',
            executable='kalman_filter_node',
            name='kalman_filter_node',
            output='screen',
            parameters=[{'noise_std': 0.8, 'enabled': True}]
        ),
        Node(
            package='turtle_game',
            executable='occupancy_grid_mapper',
            name='occupancy_grid_mapper',
            output='screen'
        ),
        Node(
            package='turtle_game',
            executable='watchdog',
            name='watchdog',
            output='screen',
            prefix='xterm -e'
        ),
        Node(
            package='turtle_game',
            executable='data_logger',
            name='data_logger',
            output='screen'
        ),
    ])
