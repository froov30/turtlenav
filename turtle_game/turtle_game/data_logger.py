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
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtle_game_interfaces.msg import GameState, TurtleType
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
import json
import os
import time
from datetime import datetime
from rich.console import Console

LOG_DIR = os.path.expanduser("~/.turtlenav_logs")


class DataLogger(Node):
    def __init__(self):
        super().__init__('data_logger')
        self.console = Console()

        self.game_state = "IDLE"
        self.session_file = None
        self.session_data = {
            "start_time":    None,
            "events":        [],
            "pose_samples":  [],
            "vel_samples":   [],
            "pid_samples":   [],
            "grid_coverage": [],
            "faults":        [],
        }
        self.sample_count = 0

        # Subscriptions — listen to everything
        self.create_subscription(
            GameState, '/game/state', self.on_game_state, 10)
        self.create_subscription(
            TurtleType, '/game/catch_type', self.on_catch, 10)
        self.create_subscription(
            Pose, '/turtle1/pose', self.on_pose, 10)
        self.create_subscription(
            Twist, '/turtle1/cmd_vel', self.on_vel, 10)
        self.create_subscription(
            String, '/system/fault', self.on_fault, 10)
        self.create_subscription(
            String, '/system/recovery', self.on_recovery, 10)
        self.create_subscription(
            OccupancyGrid, '/map/occupancy_grid', self.on_grid, 10)

        # Sample pose and velocity at 5 Hz
        self.create_timer(0.2, self.sample_loop)

        self.console.print("[bold green]Data logger ready.[/bold green]")
        self.console.print(
            f"Logs saved to: [cyan]{LOG_DIR}[/cyan]")

    # ── helpers ───────────────────────────────────────────────

    def now_sec(self):
        if self.session_data["start_time"] is None:
            return 0.0
        return time.time() - self.session_data["start_time"]

    def log_event(self, event_type, data=None):
        entry = {
            "t":    round(self.now_sec(), 3),
            "type": event_type,
        }
        if data:
            entry.update(data)
        self.session_data["events"].append(entry)

        # Also write to JSONL file immediately (real-time logging)
        if self.session_file:
            with open(self.session_file, "a") as f:
                f.write(json.dumps(entry) + "\n")

    # ── subscribers ───────────────────────────────────────────

    def on_game_state(self, msg):
        prev = self.game_state
        self.game_state = msg.state

        if msg.state == "RUNNING" and prev != "RUNNING":
            self.start_session(msg)

        elif msg.state == "GAME_OVER" and prev == "RUNNING":
            self.log_event("game_over", {
                "final_score":  msg.score,
                "catches":      len([e for e in
                                     self.session_data["events"]
                                     if e["type"] == "catch"])
            })
            self.save_session(msg)

        self.log_event("state_change", {"state": msg.state,
                                        "score": msg.score, "time": msg.time_remaining})

    def on_catch(self, msg):
        self.log_event("catch", {
            "turtle_type":  msg.turtle_type,
            "points":       msg.points_value,
        })
        self.console.print(
            f"[dim][{self.now_sec():.1f}s][/dim] "
            f"Logged catch: [cyan]{msg.turtle_type}[/cyan]"
        )

    def on_pose(self, msg):
        # Store latest pose for sampling
        self._latest_pose = msg

    def on_vel(self, msg):
        self._latest_vel = msg

    def on_fault(self, msg):
        self.log_event("fault", {"details": msg.data})
        self.console.print(
            f"[red][{self.now_sec():.1f}s] Fault logged: {msg.data}[/red]")

    def on_recovery(self, msg):
        self.log_event("recovery", {"details": msg.data})

    def on_grid(self, msg):
        if self.game_state != "RUNNING":
            return
        total = len(msg.data)
        explored = sum(1 for c in msg.data if c != -1)
        pct = round(explored / total * 100, 1) if total else 0
        self.session_data["grid_coverage"].append({
            "t":   round(self.now_sec(), 1),
            "pct": pct
        })

    def sample_loop(self):
        """Record pose and velocity samples at 5 Hz during game."""
        if self.game_state != "RUNNING":
            return
        pose = getattr(self, '_latest_pose', None)
        vel = getattr(self, '_latest_vel',  None)
        t = round(self.now_sec(), 2)

        if pose:
            self.session_data["pose_samples"].append({
                "t": t,
                "x": round(pose.x,     3),
                "y": round(pose.y,     3),
                "theta": round(pose.theta, 3)
            })
        if vel:
            self.session_data["vel_samples"].append({
                "t":       t,
                "linear":  round(vel.linear.x,  3),
                "angular": round(vel.angular.z, 3)
            })
        self.sample_count += 1

    # ── session management ────────────────────────────────────

    def start_session(self, msg):
        self.session_data = {
            "start_time":    time.time(),
            "events":        [],
            "pose_samples":  [],
            "vel_samples":   [],
            "pid_samples":   [],
            "grid_coverage": [],
            "faults":        [],
        }
        self.sample_count = 0
        self._latest_pose = None
        self._latest_vel = None

        ts = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.session_file = os.path.join(LOG_DIR, f"session_{ts}.jsonl")
        os.makedirs(LOG_DIR, exist_ok=True)

        self.log_event("game_start")
        self.console.print(
            f"[bold green]Session started → "
            f"[cyan]{self.session_file}[/cyan][/bold green]")

    def save_session(self, msg):
        if not self.session_file:
            return

        # Save full session summary JSON
        summary_path = self.session_file.replace(".jsonl", "_summary.json")
        summary = {
            "session_file":   self.session_file,
            "duration_sec":   round(self.now_sec(), 1),
            "final_score":    msg.score,
            "total_samples":  self.sample_count,
            "event_count":    len(self.session_data["events"]),
            "pose_samples":   self.session_data["pose_samples"],
            "vel_samples":    self.session_data["vel_samples"],
            "grid_coverage":  self.session_data["grid_coverage"],
            "events":         self.session_data["events"],
        }
        with open(summary_path, "w") as f:
            json.dump(summary, f, indent=2)

        self.console.print(
            f"[bold]Session saved:[/bold] [cyan]{summary_path}[/cyan]")


def main(args=None):
    rclpy.init(args=args)
    node = DataLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
