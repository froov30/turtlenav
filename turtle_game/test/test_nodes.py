# Copyright 2026 Dhruv
# SPDX-License-Identifier: MIT

"""Unit tests for pure TurtleNav control and mapping behaviour."""

import math

import numpy as np
import pytest

from turtle_game.kalman_filter import KalmanFilter1D, TurtleKalmanTracker
from turtle_game.occupancy_grid_mapper import GRID_H, GRID_RES, GRID_W
from turtle_game.occupancy_grid_mapper import OccupancyGridMapper
from turtle_game.pid_controller import PIDController
from turtle_game.target_chaser import MAX_LINEAR_SPEED, normalize_angle


def test_kalman_filter_prediction_and_update():
    """A measurement should shift the prediction toward its observed value."""
    kalman_filter = KalmanFilter1D(process_noise=0.01, measurement_noise=0.5)
    kalman_filter.predict()
    filtered_value = kalman_filter.update(np.array([6.0]))
    assert 5.5 < filtered_value < 6.0


def test_turtle_kalman_tracker_returns_filtered_coordinates():
    """The two-axis tracker returns numbers and retains positive uncertainty."""
    tracker = TurtleKalmanTracker()
    filtered_x, filtered_y = tracker.update(5.0, 5.0)
    assert isinstance(filtered_x, (float, np.floating))
    assert isinstance(filtered_y, (float, np.floating))
    assert tracker.uncertainty > 0.0


@pytest.mark.parametrize('angle', [-4.0 * math.pi, -3.5, 0.0, 3.5, 4.0 * math.pi])
def test_angle_normalization_stays_in_valid_range(angle):
    """Angle normalization handles values on either side of the wrap boundary."""
    normalized = normalize_angle(angle)
    assert -math.pi <= normalized <= math.pi


def test_autonomous_chaser_speed_limit_is_safe():
    """The configured chaser speed cap matches the documented safe limit."""
    assert min(MAX_LINEAR_SPEED, 2.0 * 10.0) == 4.0


class _Value:
    """Minimal stand-in for a ROS parameter value."""

    def __init__(self, value):
        self.double_value = value


class _Parameter:
    """Minimal stand-in for a ROS parameter."""

    def __init__(self, value):
        self.value = value

    def get_parameter_value(self):
        """Return the parameter value object expected by the controller."""
        return _Value(self.value)


class _ControllerParameters:
    """Supply deterministic gains without constructing a ROS node."""

    def __init__(self):
        self.values = {
            'kp_linear': 1.5, 'ki_linear': 0.0, 'kd_linear': 0.1,
            'kp_angular': 6.0, 'ki_angular': 0.0, 'kd_angular': 0.3,
        }

    def get_parameter(self, name):
        """Return the requested fake ROS parameter."""
        return _Parameter(self.values[name])


def test_pid_controller_reads_all_gain_parameters():
    """PID gain retrieval preserves the declared defaults and ordering."""
    controller = _ControllerParameters()
    expected = (1.5, 0.0, 0.1, 6.0, 0.0, 0.3)
    assert PIDController.get_gains(controller) == expected


@pytest.mark.parametrize(
    'x, y, expected',
    [(-1.0, -1.0, (0, 0)), (11.0, 11.0, (GRID_H - 1, GRID_W - 1)),
     (5.05, 3.01, (30, 50))],
)
def test_world_to_grid_clamps_to_valid_grid_bounds(x, y, expected):
    """World coordinates at and beyond the arena edge map to valid cells."""
    assert OccupancyGridMapper.world_to_grid(None, x, y) == expected
    assert GRID_RES == 0.1
