# Copyright 2026 Dhruv
# SPDX-License-Identifier: MIT

import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'turtle_game'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', 'turtle_game', 'launch'),
         glob('launch/*.py')),
        (os.path.join('share', 'turtle_game', 'rviz'),
         glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dhruv',
    maintainer_email='dhruv@todo.todo',
    description=('Multi-node autonomous ROS 2 target chasing and mapping game '
                 'featuring PID control, Kalman Filtering, Occupancy Grid Mapping, '
                 'and Watchdog safety monitoring.'),
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
           'turtle_spawner = turtle_game.turtle_spawner:main',
           'target_chaser = turtle_game.target_chaser:main',
           'game_manager = turtle_game.game_manager:main',
           'player_controller = turtle_game.player_controller:main',
           'pid_controller    = turtle_game.pid_controller:main',
           'kalman_filter_node = turtle_game.kalman_filter:main',
           'occupancy_grid_mapper = turtle_game.occupancy_grid_mapper:main',
           'watchdog = turtle_game.watchdog:main',
           'data_logger = turtle_game.data_logger:main',
           'analytics   = turtle_game.analytics:main',
        ],
    },
)
