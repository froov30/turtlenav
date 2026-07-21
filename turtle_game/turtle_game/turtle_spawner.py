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
from turtlesim.srv import Spawn
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import random
import math
from functools import partial
from turtle_game_interfaces.msg import TurtleData, TurtleArray
from turtle_game_interfaces.srv import KillTurtle
from turtlesim.srv import Kill
from turtle_game_interfaces.msg import TurtleType, GameState
from std_msgs.msg import Empty
from turtlesim.srv import SetPen

TURTLE_TYPES = {
    "normal":  {"weight": 50, "points": 100,  "speed": 0.0},
    "golden":  {"weight": 10, "points": 300,  "speed": 0.0, "lifetime": 5.0},
    "evasive": {"weight": 20, "points": 150,  "speed": 1.5},
    "bomb":    {"weight": 10, "points": -200, "speed": 0.0},
    "freeze":  {"weight": 10, "points": 50,   "speed": 0.0},
}


class TurtleSpawner(Node):
    def __init__(self):
        super().__init__("turtle_spawner")
        self.spawn_turtle = self.create_client(Spawn, "/spawn")
        self.turtles_publisher = self.create_publisher(TurtleArray, "turtles_data", 10)
        self.kill_turtle_service = self.create_service(
            KillTurtle, 'kill_turtle', self.kill_turtle_callback)
        self.kill_turtle_client = self.create_client(Kill, '/kill')
        self.name_counter = 1
        self.turtles_list_ = []
        self.active_turtles = {}
        self.player_pose = None
        self.evasive_publishers = {}   # name -> Twist publisher
        self.evasive_poses = {}   # name -> live Pose of that turtle
        self.evasive_subs = {}   # name -> pose subscription
        self.wander_angles = {}   # name -> current wander angle
        self.pen_clients = {}  # name -> SetPen client
        self.expiry_timers = {}  # name -> one-shot golden expiry timer
        self.pen_color_timers = {}  # name -> one-shot pen colour timer

        # evasive turtle tracking
        self.create_subscription(Pose, '/turtle1/pose', self.on_player_pose, 10)
        self.evasive_timer = self.create_timer(0.1, self.move_evasive_turtles)

        # game state
        self.game_state = "IDLE"
        self.create_subscription(GameState, '/game/state', self.on_game_state, 10)

        # catch event publishers
        self.catch_pub = self.create_publisher(Empty, '/game/catch_event', 10)
        self.type_catch_pub = self.create_publisher(TurtleType, '/game/catch_type', 10)

        # spawn timer
        self.spawn_timer = self.create_timer(2.0, self.call_spawn_service)

    def on_player_pose(self, msg):
        self.player_pose = msg

    def on_evasive_pose(self, msg, name):
        """Track live position of each evasive turtle."""
        self.evasive_poses[name] = msg
        # Also update active_turtles so proximity check stays accurate
        if name in self.active_turtles:
            self.active_turtles[name]["x"] = msg.x
            self.active_turtles[name]["y"] = msg.y

    def move_evasive_turtles(self):
        if self.player_pose is None or self.game_state != "RUNNING":
            return

        for name, info in list(self.active_turtles.items()):
            if info["type"] != "evasive":
                continue

            # Create publisher for this turtle if not exists
            if name not in self.evasive_publishers:
                self.evasive_publishers[name] = self.create_publisher(
                    Twist, f'/{name}/cmd_vel', 10)

            # Subscribe to this turtle's live pose if not already
            if name not in self.evasive_subs:
                self.evasive_subs[name] = self.create_subscription(
                    Pose, f'/{name}/pose',
                    lambda msg, n=name: self.on_evasive_pose(msg, n),
                    10)
                self.wander_angles[name] = random.uniform(0, 6.28)

            # Get live pose — skip if not yet received
            live_pose = self.evasive_poses.get(name)
            if live_pose is None:
                continue

            px = self.player_pose.x
            py = self.player_pose.y
            tx = live_pose.x
            ty = live_pose.y

            dx = tx - px
            dy = ty - py
            distance = math.sqrt(dx * dx + dy * dy)

            twist = Twist()

            if distance < 3.5:
                # Player is close — flee directly away
                angle_away = math.atan2(dy, dx)
                angle_diff = math.atan2(
                    math.sin(angle_away - live_pose.theta),
                    math.cos(angle_away - live_pose.theta)
                )
                twist.linear.x = 2.0
                twist.angular.z = 6.0 * angle_diff

            else:
                # Player is far — wander with gentle random turns
                # Change wander direction occasionally
                if random.random() < 0.02:
                    self.wander_angles[name] = random.uniform(-1.0, 1.0)

                # Wall avoidance — turn toward center if near edge
                near_wall = (tx < 1.5 or tx > 9.5 or
                             ty < 1.5 or ty > 9.5)
                if near_wall:
                    angle_to_center = math.atan2(5.5 - ty, 5.5 - tx)
                    angle_diff = math.atan2(
                        math.sin(angle_to_center - live_pose.theta),
                        math.cos(angle_to_center - live_pose.theta)
                    )
                    twist.linear.x = 1.0
                    twist.angular.z = 4.0 * angle_diff
                else:
                    twist.linear.x = 1.0
                    twist.angular.z = self.wander_angles[name]

            self.evasive_publishers[name].publish(twist)

    def on_game_state(self, msg):
        prev = self.game_state
        self.game_state = msg.state

        if msg.state == "RUNNING" and prev != "RUNNING":
            # Fresh start — clear any leftover state
            self.cancel_all_dynamic_timers()
            self.active_turtles.clear()
            self.turtles_list_.clear()
            self.evasive_publishers.clear()
            self.evasive_poses.clear()
            self.evasive_subs.clear()
            self.wander_angles.clear()
            self.get_logger().info("Game started — spawner state reset")

        elif msg.state == "GAME_OVER" and prev == "RUNNING":
            self.kill_all_turtles()

    def kill_all_turtles(self):
        names = list(self.active_turtles.keys())
        for name in names:
            self.kill_turtle_by_name(name)
        # Clear lists immediately so new game starts clean
        self.cancel_all_dynamic_timers()
        self.active_turtles.clear()
        self.turtles_list_.clear()
        self.publish_turtles_data()
        self.get_logger().info("All turtles cleared for game over")

    def kill_turtle_by_name(self, turtle_name):
        if turtle_name not in self.active_turtles:
            return
        if not self.kill_turtle_client.service_is_ready():
            self.get_logger().warn("Kill service is not available yet")
            return
        kill_request = Kill.Request()
        kill_request.name = turtle_name
        future = self.kill_turtle_client.call_async(kill_request)
        future.add_done_callback(partial(self.handle_kill_response, turtle_name=turtle_name))

    def kill_turtle_callback(self, request: KillTurtle.Request, response: KillTurtle.Response):
        turtle_name = request.name
        if turtle_name in self.active_turtles:
            t = self.active_turtles[turtle_name]
            type_msg = TurtleType()
            type_msg.name = turtle_name
            type_msg.turtle_type = t["type"]
            type_msg.points_value = TURTLE_TYPES[t["type"]]["points"]
            self.type_catch_pub.publish(type_msg)
            self.catch_pub.publish(Empty())
        response.success = self.call_kill_turtle_client(turtle_name)
        return response

    def call_kill_turtle_client(self, turtle_name):
        self.get_logger().info(f"Received request to kill turtle: {turtle_name}")
        if not self.kill_turtle_client.service_is_ready():
            self.get_logger().warn("Kill service is not available yet")
            return False
        kill_request = Kill.Request()
        kill_request.name = turtle_name
        future = self.kill_turtle_client.call_async(kill_request)
        future.add_done_callback(partial(self.handle_kill_response, turtle_name=turtle_name))
        return True

    def handle_kill_response(self, future, turtle_name):
        try:
            future.result()
        except Exception as error:  # noqa: BLE001 - ROS futures expose arbitrary errors.
            self.get_logger().error(f"Could not kill turtle {turtle_name}: {error}")
            return
        self.get_logger().info(f"Killed turtle: {turtle_name}")
        self.cancel_turtle_timers(turtle_name)
        if turtle_name in self.active_turtles:
            del self.active_turtles[turtle_name]
        # Clean up evasive turtle resources
        if turtle_name in self.evasive_publishers:
            del self.evasive_publishers[turtle_name]
        if turtle_name in self.evasive_subs:
            del self.evasive_subs[turtle_name]
        if turtle_name in self.evasive_poses:
            del self.evasive_poses[turtle_name]
        if turtle_name in self.wander_angles:
            del self.wander_angles[turtle_name]
        for i, turtle in enumerate(self.turtles_list_):
            if turtle.name == turtle_name:
                self.turtles_list_.pop(i)
                self.publish_turtles_data()
                break

    def call_spawn_service(self):
        if self.game_state != "RUNNING":
            return
        types = list(TURTLE_TYPES.keys())
        weights = [TURTLE_TYPES[t]["weight"] for t in types]
        chosen_type = random.choices(types, weights=weights, k=1)[0]
        self.name_counter += 1
        x = random.uniform(0.5, 10.5)
        y = random.uniform(0.5, 10.5)
        theta = random.uniform(0.0, 6.28)
        name = "turtle" + str(self.name_counter)
        self.send_spawn_request(x, y, theta, name, chosen_type)

    def send_spawn_request(self, x, y, theta, name, chosen_type):
        if not self.spawn_turtle.service_is_ready():
            self.get_logger().warn("Spawn service is not available yet")
            return
        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        request.name = name
        future = self.spawn_turtle.call_async(request)
        future.add_done_callback(
            partial(self.handle_response, request=request, chosen_type=chosen_type))

    def handle_response(self, future, request, chosen_type):
        response = future.result()
        if response.name != "":
            self.get_logger().info(f"Spawned [{chosen_type}] turtle: {response.name}")
            data = TurtleData()
            data.name = response.name
            data.x = request.x
            data.y = request.y
            data.theta = request.theta
            self.turtles_list_.append(data)
            self.active_turtles[response.name] = {
                "type": chosen_type,
                "x": request.x,
                "y": request.y,
                "spawn_time": self.get_clock().now().nanoseconds / 1e9
            }
            if chosen_type == "golden":
                self.expiry_timers[response.name] = self.create_timer(
                    5.0, partial(self.expire_turtle, response.name))
                self.pen_color_timers[response.name] = self.create_timer(
                    0.5, partial(self.set_turtle_pen_color, response.name, chosen_type))
            self.publish_turtles_data()

    def expire_turtle(self, name):
        timer = self.expiry_timers.pop(name, None)
        if timer is not None:
            timer.cancel()
        if name in self.active_turtles:
            self.get_logger().info(f"Golden turtle {name} expired")
            self.kill_turtle_by_name(name)

    def publish_turtles_data(self):
        msg = TurtleArray()
        msg.turtles = self.turtles_list_
        self.turtles_publisher.publish(msg)

    def set_turtle_pen_color(self, name, turtle_type):
        """Give each turtle type a distinctive pen color trail."""
        timer = self.pen_color_timers.pop(name, None)
        if timer is not None:
            timer.cancel()
        type_colors = {
            'normal':  (255, 255, 255, 2),
            'golden':  (255, 215,   0, 3),
            'evasive': (0, 200, 255, 2),
            'bomb':    (255,  50,  50, 3),
            'freeze':  (50, 100, 255, 3),
        }
        if turtle_type not in type_colors:
            return
        r, g, b, w = type_colors[turtle_type]
        service_name = f'/{name}/set_pen'
        if name not in self.pen_clients:
            self.pen_clients[name] = self.create_client(SetPen, service_name)
        client = self.pen_clients[name]
        if not client.wait_for_service(timeout_sec=0.5):
            return
        req = SetPen.Request()
        req.r = r
        req.g = g
        req.b = b
        req.width = w
        req.off = 0
        client.call_async(req)

    def cancel_turtle_timers(self, turtle_name):
        """Cancel and discard one-shot timers associated with a turtle."""
        for timers in (self.expiry_timers, self.pen_color_timers):
            timer = timers.pop(turtle_name, None)
            if timer is not None:
                timer.cancel()

    def cancel_all_dynamic_timers(self):
        """Cancel all turtle-specific timers before resetting game state."""
        names = set(self.expiry_timers) | set(self.pen_color_timers)
        for name in names:
            self.cancel_turtle_timers(name)


def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
