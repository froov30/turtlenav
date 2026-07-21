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
from turtlesim.srv import SetPen, TeleportAbsolute
from std_msgs.msg import Empty
from std_srvs.srv import Empty as EmptySrv
from turtle_game_interfaces.msg import GameState, TurtleArray
from turtle_game_interfaces.srv import KillTurtle
from functools import partial
import sys
import tty
import termios
import threading
import math
from rich.console import Console

LINEAR_SPEED = 4.0
ANGULAR_SPEED = 4.0

KEY_BINDINGS = {
    'w': (LINEAR_SPEED,  0.0),
    's': (-LINEAR_SPEED,  0.0),
    'a': (0.0,  ANGULAR_SPEED),
    'd': (0.0, -ANGULAR_SPEED),
    ' ': (0.0,  0.0),
}

TYPE_COLORS = {
    'normal':  [255, 255, 255, 2],
    'golden':  [255, 215,   0, 3],
    'evasive': [0, 200, 255, 2],
    'bomb':    [255,  50,  50, 3],
    'freeze':  [50,  50, 255, 3],
}


class PlayerController(Node):
    def __init__(self):
        super().__init__('player_controller')
        self.console = Console()

        self.player_pose = None
        self.game_state = "IDLE"
        self.turtles_data = []
        self.killing = set()
        self.start_requested = False

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel',    10)
        self.start_req_pub = self.create_publisher(Empty, '/game/start_request', 10)

        # Subscribers
        self.create_subscription(Pose,       '/turtle1/pose', self.on_pose,       10)
        self.create_subscription(GameState,  '/game/state',   self.on_game_state, 10)
        self.create_subscription(TurtleArray, '/turtles_data_filtered',  self.on_turtles,    10)

        # Service clients
        self.kill_client = self.create_client(KillTurtle,      'kill_turtle')
        self.teleport_client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        self.pen_client_main = self.create_client(SetPen,          '/turtle1/set_pen')
        self.clear_client = self.create_client(EmptySrv,        '/clear')

        # Pen clients cache for target turtles
        self.pen_clients = {}

        # Timers
        self.create_timer(0.05, self.proximity_check)
        self.create_timer(0.1,  self.check_start_request)

        # Key reading thread
        self.running = True
        self.key_thread = threading.Thread(target=self.read_keys, daemon=True)
        self.key_thread.start()

        self.console.print("[bold green]Player controller ready.[/bold green]")
        self.console.print(
            "  [cyan]W/A/S/D[/cyan] move   [cyan]SPACE[/cyan] stop   "
            "[cyan]S (capital)[/cyan] start   [cyan]Ctrl+C[/cyan] quit")

    # ── callbacks ──────────────────────────────────────────────

    def on_pose(self, msg):
        self.player_pose = msg

    def on_game_state(self, msg):
        prev = self.game_state
        self.game_state = msg.state
        if msg.state == "GAME_OVER" and prev == "RUNNING":
            self.reset_turtle()

    def on_turtles(self, msg):
        self.turtles_data = msg.turtles

    # ── start request ──────────────────────────────────────────

    def check_start_request(self):
        if self.start_requested:
            self.start_requested = False
            self.start_req_pub.publish(Empty())
            self.console.print("[bold yellow]Start request sent![/bold yellow]")

    # ── proximity / kill ───────────────────────────────────────

    def proximity_check(self):
        if self.player_pose is None or self.game_state != "RUNNING":
            return
        for turtle in self.turtles_data:
            if turtle.name in self.killing:
                continue
            dx = turtle.x - self.player_pose.x
            dy = turtle.y - self.player_pose.y
            if math.sqrt(dx * dx + dy * dy) < 0.8:
                self.killing.add(turtle.name)
                self.call_kill_service(turtle.name)

    def call_kill_service(self, turtle_name):
        if not self.kill_client.wait_for_service(timeout_sec=1.0):
            self.killing.discard(turtle_name)
            return
        req = KillTurtle.Request()
        req.name = turtle_name
        future = self.kill_client.call_async(req)
        future.add_done_callback(
            partial(self.on_kill_response, turtle_name=turtle_name))

    def on_kill_response(self, future, turtle_name):
        try:
            future.result()
            self.get_logger().info(f"Caught: {turtle_name}")
        except Exception as e:
            self.get_logger().warn(f"Kill failed for {turtle_name}: {e}")
        finally:
            self.killing.discard(turtle_name)

    # ── reset after game over ──────────────────────────────────

    def reset_turtle(self):
        pen_off = SetPen.Request()
        pen_off.r = 0
        pen_off.g = 0
        pen_off.b = 0
        pen_off.width = 0
        pen_off.off = 1
        future = self.pen_client_main.call_async(pen_off)
        future.add_done_callback(lambda f: self.do_teleport())

    def do_teleport(self):
        req = TeleportAbsolute.Request()
        req.x = 5.544445
        req.y = 5.544445
        req.theta = 0.0
        future = self.teleport_client.call_async(req)
        future.add_done_callback(lambda f: self.clear_background())

    def clear_background(self):
        self.clear_client.call_async(EmptySrv.Request())
        pen_on = SetPen.Request()
        pen_on.r = 185
        pen_on.g = 185
        pen_on.b = 185
        pen_on.width = 3
        pen_on.off = 0
        self.pen_client_main.call_async(pen_on)
        self.console.print("[bold green]Arena cleared. Press S to play again.[/bold green]")

    # ── key reading ────────────────────────────────────────────

    def read_keys(self):
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            while self.running:
                key = sys.stdin.read(1)
                if key == '\x03':
                    self.running = False
                    break
                if key == 'S':
                    self.start_requested = True
                    continue
                if self.game_state != "RUNNING":
                    continue
                if key in KEY_BINDINGS:
                    lin, ang = KEY_BINDINGS[key]
                    twist = Twist()
                    twist.linear.x = lin
                    twist.angular.z = ang
                    self.cmd_pub.publish(twist)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    def destroy_node(self):
        self.running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PlayerController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
