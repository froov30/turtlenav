# MIT License
# Copyright (c) 2026 Dhruv
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import rclpy
from rclpy.node import Node
from turtle_game_interfaces.msg import TurtleArray, TurtleData, GameState
import numpy as np
import random
from rich.console import Console


class KalmanFilter1D:
    """
    Independent 1D Kalman filter for x and y position.

    State: [position, velocity]
    """

    def __init__(self, process_noise=0.1, measurement_noise=1.5):
        # State estimate [x, v]
        self.x = np.array([5.5, 0.0])

        # State covariance — starts uncertain
        self.P = np.eye(2) * 2.0

        # State transition matrix (constant velocity model)
        self.dt = 0.1
        self.F = np.array([
            [1, self.dt],
            [0, 1]
        ])

        # Measurement matrix (we only measure position, not velocity)
        self.H = np.array([[1, 0]])

        # Process noise covariance
        self.Q = np.eye(2) * process_noise

        # Measurement noise covariance — how noisy the sensor is
        self.R = np.array([[measurement_noise]])

    def predict(self):
        """Predict step — move state forward in time."""
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, measurement):
        """Update step — fuse measurement with prediction."""
        y = measurement - self.H @ self.x          # innovation
        S = self.H @ self.P @ self.H.T + self.R    # innovation covariance
        K = self.P @ self.H.T @ np.linalg.inv(S)  # Kalman gain
        self.x = self.x + K @ y                    # state update
        self.P = (np.eye(2) - K @ self.H) @ self.P  # covariance update
        return self.x[0]  # return filtered position

    @property
    def uncertainty(self):
        return float(self.P[0, 0])  # position variance


class TurtleKalmanTracker:
    """Tracks one turtle with separate KF for x and y."""

    def __init__(self):
        self.kf_x = KalmanFilter1D(process_noise=0.05, measurement_noise=1.2)
        self.kf_y = KalmanFilter1D(process_noise=0.05, measurement_noise=1.2)

    def update(self, noisy_x, noisy_y):
        self.kf_x.predict()
        self.kf_y.predict()
        fx = self.kf_x.update(np.array([noisy_x]))
        fy = self.kf_y.update(np.array([noisy_y]))
        return fx, fy

    @property
    def uncertainty(self):
        return (self.kf_x.uncertainty + self.kf_y.uncertainty) / 2.0


class KalmanFilterNode(Node):
    def __init__(self):
        super().__init__('kalman_filter_node')
        self.console = Console()

        # Noise level — tuneable via parameter
        self.declare_parameter('noise_std', 0.8)
        self.declare_parameter('enabled',   True)

        self.game_state = "IDLE"
        self.trackers = {}  # name -> TurtleKalmanTracker
        self.raw_turtles = []

        # Subscribe to raw turtle data from spawner
        self.create_subscription(
            TurtleArray, 'turtles_data', self.on_turtles, 10)
        self.create_subscription(
            GameState, '/game/state', self.on_game_state, 10)

        # Publish filtered turtle positions on a separate topic
        # Player controller can optionally subscribe to this instead
        self.filtered_pub = self.create_publisher(
            TurtleArray, '/turtles_data_filtered', 10)

        # Also publish noise stats for visualization
        self.create_timer(0.1, self.filter_loop)
        self.create_timer(2.0, self.log_stats)

        self.console.print("[bold yellow]Kalman filter node ready.[/bold yellow]")
        self.console.print(
            "Tune noise: [cyan]ros2 param set /kalman_filter_node noise_std 1.5[/cyan]")

    def on_game_state(self, msg):
        self.game_state = msg.state
        if msg.state == "IDLE" or msg.state == "GAME_OVER":
            self.trackers.clear()

    def on_turtles(self, msg):
        self.raw_turtles = msg.turtles
        # Clean up trackers for turtles that no longer exist
        active_names = {t.name for t in msg.turtles}
        for name in list(self.trackers.keys()):
            if name not in active_names:
                del self.trackers[name]

    def filter_loop(self):
        if not self.get_parameter('enabled').get_parameter_value().bool_value:
            return
        if self.game_state != "RUNNING":
            return

        noise_std = self.get_parameter('noise_std').get_parameter_value().double_value
        filtered_msg = TurtleArray()

        for turtle in self.raw_turtles:
            # Add Gaussian noise to simulate a real noisy sensor
            noisy_x = turtle.x + random.gauss(0, noise_std)
            noisy_y = turtle.y + random.gauss(0, noise_std)

            # Clamp to arena bounds
            noisy_x = max(0.5, min(10.5, noisy_x))
            noisy_y = max(0.5, min(10.5, noisy_y))

            # Create or update Kalman tracker
            if turtle.name not in self.trackers:
                self.trackers[turtle.name] = TurtleKalmanTracker()
                # Initialize tracker at noisy position
                self.trackers[turtle.name].kf_x.x[0] = noisy_x
                self.trackers[turtle.name].kf_y.x[0] = noisy_y

            tracker = self.trackers[turtle.name]
            fx, fy = tracker.update(noisy_x, noisy_y)

            # Build filtered TurtleData message
            filtered = TurtleData()
            filtered.name = turtle.name
            filtered.x = float(fx)
            filtered.y = float(fy)
            filtered.theta = turtle.theta
            filtered_msg.turtles.append(filtered)

        self.filtered_pub.publish(filtered_msg)

    def log_stats(self):
        if not self.trackers:
            return
        for name, tracker in self.trackers.items():
            self.console.print(
                f"[dim]{name}[/dim] uncertainty: "
                f"[cyan]{tracker.uncertainty:.3f}[/cyan] "
                f"(lower = more confident)"
            )


def main(args=None):
    rclpy.init(args=args)
    node = KalmanFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
