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

"""Autonomously chase and remove turtles published by the spawner."""

from functools import partial
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from turtlesim.msg import Pose

from turtle_game_interfaces.msg import TurtleArray
from turtle_game_interfaces.srv import KillTurtle

MAX_LINEAR_SPEED = 4.0


def normalize_angle(angle):
    """Normalize an angle to the inclusive range [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


class TargetChaser(Node):
    """Drive turtle1 toward the first active target in autonomous mode."""

    def __init__(self):
        super().__init__('target_chaser')
        self.pose = None
        self.target = None
        self.cmd_vel_publisher = self.create_publisher(
            Twist, '/turtle1/cmd_vel', 10)
        self.create_subscription(Pose, '/turtle1/pose', self.on_pose, 10)
        self.create_subscription(
            TurtleArray, '/turtles_data', self.on_turtles, 10)
        self.kill_turtle_client = self.create_client(KillTurtle, '/kill_turtle')
        self.create_timer(0.05, self.controller_loop)

    def on_pose(self, msg):
        """Store turtle1's latest pose."""
        self.pose = msg

    def on_turtles(self, msg):
        """Select the first available target, or clear a stale target."""
        self.target = msg.turtles[0] if msg.turtles else None

    def controller_loop(self):
        """Publish a bounded proportional velocity command toward the target."""
        if self.pose is None or self.target is None:
            return

        cmd = Twist()
        dx = self.target.x - self.pose.x
        dy = self.target.y - self.pose.y
        distance = math.hypot(dx, dy)

        if distance > 0.3:
            target_angle = normalize_angle(math.atan2(dy, dx) - self.pose.theta)
            cmd.linear.x = min(MAX_LINEAR_SPEED, 2.0 * distance)
            cmd.angular.z = 6.0 * target_angle
        else:
            self.call_kill_turtle_service(self.target.name)
            self.target = None

        self.cmd_vel_publisher.publish(cmd)

    def call_kill_turtle_service(self, turtle_name):
        """Request a kill without blocking the control loop for service discovery."""
        if not self.kill_turtle_client.service_is_ready():
            self.get_logger().warn('KillTurtle service is not available yet')
            return

        request = KillTurtle.Request()
        request.name = turtle_name
        future = self.kill_turtle_client.call_async(request)
        future.add_done_callback(
            partial(self.handle_kill_response, turtle_name=turtle_name))

    def handle_kill_response(self, future, turtle_name):
        """Log the asynchronous kill result."""
        try:
            response = future.result()
            self.get_logger().info(
                f'Kill response for turtle {turtle_name}: {response.success}')
        except Exception as error:  # noqa: BLE001 - ROS futures expose arbitrary errors.
            self.get_logger().error(f'Could not kill {turtle_name}: {error}')


def main(args=None):
    """Run the autonomous target chaser node."""
    rclpy.init(args=args)
    node = TargetChaser()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
