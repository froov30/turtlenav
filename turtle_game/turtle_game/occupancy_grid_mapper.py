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
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from turtle_game_interfaces.msg import TurtleType, GameState
import numpy as np
from rich.console import Console

ARENA_SIZE = 11.0
GRID_RES = 0.1
GRID_W = int(ARENA_SIZE / GRID_RES)
GRID_H = int(ARENA_SIZE / GRID_RES)

UNKNOWN = -1
FREE = 0
EXPLORED = 10
HIGH_VALUE = 40
DANGER = 90


class OccupancyGridMapper(Node):
    def __init__(self):
        super().__init__('occupancy_grid_mapper')
        self.console = Console()

        self.grid = np.full((GRID_H, GRID_W), UNKNOWN, dtype=np.int8)
        self.player_pose = None
        self.game_state = "IDLE"

        # Path tracking — list of (x, y) positions
        self.path_points = []
        self.bomb_catches = []   # list of (x, y)
        self.golden_catches = []   # list of (x, y)
        self.marker_id = 0

        # Subscriptions
        self.create_subscription(
            Pose,      '/turtle1/pose',    self.on_pose,       10)
        self.create_subscription(
            TurtleType, '/game/catch_type', self.on_catch,      10)
        self.create_subscription(
            GameState, '/game/state',      self.on_game_state, 10)

        # Publishers
        self.grid_pub = self.create_publisher(
            OccupancyGrid, '/map/occupancy_grid', 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, '/map/markers', 10)
        self.path_pub = self.create_publisher(
            Marker, '/map/path', 10)

        self.create_timer(0.2, self.update_map)
        self.create_timer(0.5, self.publish_all)
        self.create_timer(10.0, self.log_stats)

        self.console.print("[bold blue]Occupancy grid mapper ready.[/bold blue]")
        self.console.print(
            "In RViz2 add:\n"
            "  [cyan]/map/occupancy_grid[/cyan] (Map)\n"
            "  [cyan]/map/path[/cyan] (Marker)\n"
            "  [cyan]/map/markers[/cyan] (MarkerArray)"
        )

    def on_pose(self, msg):
        self.player_pose = msg
        if self.game_state == "RUNNING":
            # Record path point every 0.2s (called by update_map timer)
            self.path_points.append((msg.x, msg.y))
            # Keep last 5000 points max
            if len(self.path_points) > 5000:
                self.path_points.pop(0)

    def on_game_state(self, msg):
        prev = self.game_state
        self.game_state = msg.state
        if msg.state == "RUNNING" and prev != "RUNNING":
            # Reset everything for new game
            self.grid = np.full(
                (GRID_H, GRID_W), UNKNOWN, dtype=np.int8)
            self.path_points = []
            self.bomb_catches = []
            self.golden_catches = []
            self.console.print("[dim]Grid reset for new game[/dim]")

    def world_to_grid(self, x, y):
        col = int(x / GRID_RES)
        row = int(y / GRID_RES)
        col = max(0, min(GRID_W - 1, col))
        row = max(0, min(GRID_H - 1, row))
        return row, col

    def mark_circle(self, cx, cy, radius_cells, value):
        row, col = self.world_to_grid(cx, cy)
        for dr in range(-radius_cells, radius_cells + 1):
            for dc in range(-radius_cells, radius_cells + 1):
                if dr*dr + dc*dc <= radius_cells*radius_cells:
                    r = max(0, min(GRID_H-1, row+dr))
                    c = max(0, min(GRID_W-1, col+dc))
                    if value in [HIGH_VALUE, DANGER]:
                        self.grid[r, c] = value
                    elif self.grid[r, c] not in [HIGH_VALUE, DANGER]:
                        self.grid[r, c] = value

    def update_map(self):
        if self.player_pose is None or self.game_state != "RUNNING":
            return
        self.mark_circle(
            self.player_pose.x,
            self.player_pose.y,
            radius_cells=4,
            value=EXPLORED
        )

    def on_catch(self, msg):
        if self.game_state != "RUNNING" or self.player_pose is None:
            return
        x, y = self.player_pose.x, self.player_pose.y

        if msg.turtle_type == "bomb":
            self.bomb_catches.append((x, y))
            self.mark_circle(x, y, radius_cells=8, value=DANGER)
            self.console.print(
                f"[red]Danger zone marked ({x:.1f},{y:.1f})[/red]")

        elif msg.turtle_type == "golden":
            self.golden_catches.append((x, y))
            self.mark_circle(x, y, radius_cells=6, value=HIGH_VALUE)
            self.console.print(
                f"[yellow]High-value zone ({x:.1f},{y:.1f})[/yellow]")

    def publish_all(self):
        self.publish_grid()
        self.publish_path()
        self.publish_markers()

    def publish_grid(self):
        msg = OccupancyGrid()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.info.resolution = GRID_RES
        msg.info.width = GRID_W
        msg.info.height = GRID_H
        msg.info.origin.position.x = 0.0
        msg.info.origin.position.y = 0.0
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = self.grid.flatten().tolist()
        self.grid_pub.publish(msg)

    def publish_path(self):
        """Publish the turtle's movement path as a line strip marker."""
        if len(self.path_points) < 2:
            return

        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'turtle_path'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        # Line width in meters
        marker.scale.x = 0.05

        # Cyan color for path
        marker.color.r = 0.0
        marker.color.g = 0.8
        marker.color.b = 1.0
        marker.color.a = 0.8

        # Add all path points
        for x, y in self.path_points:
            p = Point()
            p.x = x
            p.y = y
            p.z = 0.0
            marker.points.append(p)

        self.path_pub.publish(marker)

    def publish_markers(self):
        """Publish sphere markers for bomb and golden catch locations."""
        marker_array = MarkerArray()
        mid = 0

        # Bomb catch markers — red spheres
        for x, y in self.bomb_catches:
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'bomb_catches'
            m.id = mid
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = 0.0
            m.pose.orientation.w = 1.0
            m.scale.x = 0.4
            m.scale.y = 0.4
            m.scale.z = 0.4
            m.color.r = 1.0
            m.color.g = 0.2
            m.color.b = 0.2
            m.color.a = 0.9
            marker_array.markers.append(m)
            mid += 1

        # Golden catch markers — yellow spheres
        for x, y in self.golden_catches:
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'golden_catches'
            m.id = mid
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = 0.0
            m.pose.orientation.w = 1.0
            m.scale.x = 0.4
            m.scale.y = 0.4
            m.scale.z = 0.4
            m.color.r = 1.0
            m.color.g = 0.85
            m.color.b = 0.0
            m.color.a = 0.9
            marker_array.markers.append(m)
            mid += 1

        if marker_array.markers:
            self.marker_pub.publish(marker_array)

    def log_stats(self):
        total = GRID_W * GRID_H
        explored = int(np.sum(self.grid == EXPLORED))
        danger = int(np.sum(self.grid == DANGER))
        highval = int(np.sum(self.grid == HIGH_VALUE))
        unknown = int(np.sum(self.grid == UNKNOWN))
        pct = (explored + danger + highval) / total * 100
        self.console.print(
            f"[dim]Map:[/dim] [cyan]{pct:.1f}% explored[/cyan]  "
            f"[yellow]{highval} high-value[/yellow]  "
            f"[red]{danger} danger[/red]  "
            f"[dim]{unknown} unknown[/dim]  "
            f"path: [cyan]{len(self.path_points)} pts[/cyan]"
        )


def main(args=None):
    rclpy.init(args=args)
    node = OccupancyGridMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
