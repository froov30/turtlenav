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
from std_msgs.msg import String
from turtle_game_interfaces.msg import TurtleArray, GameState
from rich.console import Console
from rich.table import Table
from rich import box as rich_box
import time

# How long (seconds) before a node is considered dead
TIMEOUT = 3.0

# Nodes the watchdog monitors and which topic proves they are alive
MONITORED = {
    'turtlesim':             '/turtle1/pose',
    'turtle_spawner':        '/turtles_data',
    'game_manager':          '/game/state',
    'player_controller':     '/turtle1/cmd_vel',
    'kalman_filter_node':    '/turtles_data_filtered',
    'occupancy_grid_mapper': '/map/occupancy_grid',
}


class Watchdog(Node):
    def __init__(self):
        super().__init__('watchdog')
        self.console = Console()

        # Last heartbeat time for each monitored node
        self.last_seen = {name: time.time() for name in MONITORED}
        self.fault_active = {name: False for name in MONITORED}
        self.game_state = "IDLE"

        # Fault publisher — any node can subscribe to react to faults
        self.fault_pub = self.create_publisher(String, '/system/fault', 10)
        self.recovery_pub = self.create_publisher(String, '/system/recovery', 10)

        # Subscribe to each node's heartbeat topic using generic callbacks
        from turtlesim.msg import Pose as TurtlePose
        from nav_msgs.msg import OccupancyGrid

        self.create_subscription(
            TurtlePose, '/turtle1/pose',
            lambda msg: self.heartbeat('turtlesim'), 10)
        self.create_subscription(
            TurtleArray, '/turtles_data',
            lambda msg: self.heartbeat('turtle_spawner'), 10)
        self.create_subscription(
            GameState, '/game/state',
            lambda msg: self.on_game_state(msg), 10)
        self.create_subscription(
            TurtleArray, '/turtles_data_filtered',
            lambda msg: self.heartbeat('kalman_filter_node'), 10)
        self.create_subscription(
            OccupancyGrid, '/map/occupancy_grid',
            lambda msg: self.heartbeat('occupancy_grid_mapper'), 10)

        # Monitor loop at 1 Hz
        self.create_timer(1.0, self.check_health)

        # Status display at 5 seconds
        self.create_timer(5.0, self.display_status)

        self.console.print("[bold red]Watchdog node active.[/bold red]")
        self.console.print(
            f"Monitoring [cyan]{len(MONITORED)}[/cyan] nodes. "
            f"Fault timeout: [yellow]{TIMEOUT}s[/yellow]")

    def heartbeat(self, node_name):
        """Record a heartbeat whenever a monitored node publishes."""
        now = time.time()
        self.last_seen[node_name] = now

        # If node was previously faulted, publish recovery
        if self.fault_active.get(node_name, False):
            self.fault_active[node_name] = False
            msg = String()
            msg.data = f"RECOVERED:{node_name}"
            self.recovery_pub.publish(msg)
            self.console.print(
                f"[bold green]RECOVERED[/bold green] — "
                f"[cyan]{node_name}[/cyan] is back online")

    def on_game_state(self, msg):
        self.heartbeat('game_manager')
        self.game_state = msg.state

    def check_health(self):
        """Check every monitored node for missed heartbeats."""
        now = time.time()
        # Only monitor actively during RUNNING state
        # (nodes naturally go quiet in IDLE)
        if self.game_state not in ["RUNNING", "COUNTDOWN"]:
            return

        for node_name, last_time in self.last_seen.items():
            elapsed = now - last_time
            if elapsed > TIMEOUT:
                if not self.fault_active.get(node_name, False):
                    # New fault detected
                    self.fault_active[node_name] = True
                    self.publish_fault(node_name, elapsed)

    def publish_fault(self, node_name, elapsed):
        """Publish fault event and log it prominently."""
        msg = String()
        msg.data = f"FAULT:{node_name}:timeout:{elapsed:.1f}s"
        self.fault_pub.publish(msg)

        self.console.print(
            f"\n[bold red]FAULT DETECTED[/bold red] — "
            f"[yellow]{node_name}[/yellow] "
            f"has not published for [red]{elapsed:.1f}s[/red]"
        )
        self.console.print(
            f"[dim]Published to /system/fault: {msg.data}[/dim]"
        )

        # In a real safety-critical system you would trigger
        # an emergency stop here. For the game we just log.
        if node_name == 'turtlesim':
            self.console.print(
                "[bold red]CRITICAL: Simulation lost. "
                "All nodes should stop.[/bold red]")

    def display_status(self):
        """Print a periodic health table to the terminal."""
        now = time.time()
        table = Table(
            title="[bold]System health[/bold]",
            box=rich_box.SIMPLE_HEAVY,
            show_header=True
        )
        table.add_column("Node",    style="cyan",  width=24)
        table.add_column("Status",  width=10)
        table.add_column("Last seen", width=12)
        table.add_column("Topic",   style="dim",   width=28)

        for node_name, topic in MONITORED.items():
            elapsed = now - self.last_seen[node_name]
            faulted = self.fault_active.get(node_name, False)

            if faulted:
                status = "[red]FAULT[/red]"
                seen = f"[red]{elapsed:.0f}s ago[/red]"
            elif elapsed < TIMEOUT:
                status = "[green]OK[/green]"
                seen = f"[dim]{elapsed:.1f}s ago[/dim]"
            else:
                status = "[yellow]IDLE[/yellow]"
                seen = f"[dim]{elapsed:.0f}s ago[/dim]"

            table.add_row(node_name, status, seen, topic)

        self.console.print(table)


def main(args=None):
    rclpy.init(args=args)
    node = Watchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
