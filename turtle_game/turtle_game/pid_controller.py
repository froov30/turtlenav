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
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtle_game_interfaces.msg import TurtleArray, GameState
import math
from rich.console import Console


class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller')
        self.console = Console()

        # PID gains — declare as ROS2 parameters so they can be tuned live
        self.declare_parameter('kp_linear',  1.5)
        self.declare_parameter('ki_linear',  0.0)
        self.declare_parameter('kd_linear',  0.1)
        self.declare_parameter('kp_angular', 6.0)
        self.declare_parameter('ki_angular', 0.0)
        self.declare_parameter('kd_angular', 0.3)
        self.declare_parameter('enabled',    False)  # only active when game runs

        # PID state
        self.prev_linear_error = 0.0
        self.prev_angular_error = 0.0
        self.integral_linear = 0.0
        self.integral_angular = 0.0
        self.last_time = None

        # Robot state
        self.pose = None
        self.target = None
        self.game_state = "IDLE"

        # Publishers / subscribers
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        self.create_subscription(Pose,       '/turtle1/pose', self.on_pose,       10)
        self.create_subscription(TurtleArray, 'turtles_data',  self.on_turtles,    10)
        self.create_subscription(GameState,  '/game/state',   self.on_game_state, 10)

        # Control loop at 20 Hz
        self.create_timer(0.05, self.control_loop)

        # Parameter update check at 1 Hz
        self.create_timer(1.0, self.log_gains)

        self.console.print("[bold purple]PID controller ready.[/bold purple]")
        self.console.print(
            "Tune gains live: [cyan]ros2 param set /pid_controller kp_linear 2.0[/cyan]")

    def on_pose(self, msg):
        self.pose = msg

    def on_game_state(self, msg):
        self.game_state = msg.state
        if msg.state != "RUNNING":
            # reset integrators when not running to prevent windup
            self.integral_linear = 0.0
            self.integral_angular = 0.0
            self.last_time = None

    def on_turtles(self, msg):
        if len(msg.turtles) > 0:
            self.target = msg.turtles[0]
        else:
            self.target = None

    def control_loop(self):
        enabled = self.get_parameter('enabled').get_parameter_value().bool_value
        if not enabled:
            return
        if self.pose is None or self.target is None:
            return
        if self.game_state != "RUNNING":
            return

        now = self.get_clock().now().nanoseconds / 1e9
        if self.last_time is None:
            self.last_time = now
            return
        dt = now - self.last_time
        if dt <= 0.0:
            return
        self.last_time = now

        kp_lin, ki_lin, kd_lin, kp_ang, ki_ang, kd_ang = self.get_gains()

        # Compute errors
        dx = self.target.x - self.pose.x
        dy = self.target.y - self.pose.y
        linear_error = math.sqrt(dx * dx + dy * dy)
        desired_angle = math.atan2(dy, dx)
        angular_error = math.atan2(
            math.sin(desired_angle - self.pose.theta),
            math.cos(desired_angle - self.pose.theta)
        )

        if linear_error < 0.5:
            # close enough — stop and let proximity check handle kill
            twist = Twist()
            self.cmd_pub.publish(twist)
            self.integral_linear = 0.0
            self.integral_angular = 0.0
            return

        # Integral terms (with anti-windup clamp)
        self.integral_linear = max(-5.0, min(5.0,
                                             self.integral_linear + linear_error * dt))
        self.integral_angular = max(-2.0, min(2.0,
                                              self.integral_angular + angular_error * dt))

        # Derivative terms
        d_linear = (linear_error - self.prev_linear_error) / dt
        d_angular = (angular_error - self.prev_angular_error) / dt

        self.prev_linear_error = linear_error
        self.prev_angular_error = angular_error

        # PID output
        linear_vel = (kp_lin * linear_error
                      + ki_lin * self.integral_linear
                      + kd_lin * d_linear)
        angular_vel = (kp_ang * angular_error
                       + ki_ang * self.integral_angular
                       + kd_ang * d_angular)

        # Clamp velocities
        linear_vel = max(-3.0, min(3.0,  linear_vel))
        angular_vel = max(-4.0, min(4.0, angular_vel))

        twist = Twist()
        twist.linear.x = linear_vel
        twist.angular.z = angular_vel
        self.cmd_pub.publish(twist)

    def get_gains(self):
        """Return the six live-tunable PID gains in control-loop order."""
        kp_lin = self.get_parameter('kp_linear').get_parameter_value().double_value
        ki_lin = self.get_parameter('ki_linear').get_parameter_value().double_value
        kd_lin = self.get_parameter('kd_linear').get_parameter_value().double_value
        kp_ang = self.get_parameter('kp_angular').get_parameter_value().double_value
        ki_ang = self.get_parameter('ki_angular').get_parameter_value().double_value
        kd_ang = self.get_parameter('kd_angular').get_parameter_value().double_value
        return kp_lin, ki_lin, kd_lin, kp_ang, ki_ang, kd_ang

    def log_gains(self):
        kp = self.get_parameter('kp_linear').get_parameter_value().double_value
        ki = self.get_parameter('ki_linear').get_parameter_value().double_value
        kd = self.get_parameter('kd_linear').get_parameter_value().double_value
        self.console.print(
            f"[dim]PID gains — Kp:[/dim] [cyan]{kp}[/cyan]  "
            f"[dim]Ki:[/dim] [cyan]{ki}[/cyan]  "
            f"[dim]Kd:[/dim] [cyan]{kd}[/cyan]"
        )


def main(args=None):
    rclpy.init(args=args)
    node = PIDController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
