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
from turtle_game_interfaces.msg import GameState
from rich.console import Console
from rich.table import Table
from rich.text import Text
from rich.panel import Panel
from rich import box as rich_box
import json
import os
import glob
from collections import defaultdict

LOG_DIR = os.path.expanduser("~/.turtlenav_logs")

SPARK_CHARS = " ▁▂▃▄▅▆▇█"


def sparkline(values, width=30):
    if not values:
        return "no data"
    max_v = max(values) if max(values) > 0 else 1
    result = ""
    for v in values[-width:]:
        idx = int((v / max_v) * (len(SPARK_CHARS) - 1))
        result += SPARK_CHARS[min(idx, len(SPARK_CHARS)-1)]
    return result


def bar(value, max_val, width=20, color="cyan"):
    ratio = min(abs(value) / max(max_val, 0.001), 1.0)
    filled = int(ratio * width)
    return "█" * filled + "░" * (width - filled)


class Analytics(Node):
    def __init__(self):
        super().__init__('analytics')
        self.console = Console()

        # Wait for game over then run analytics
        self.create_subscription(
            GameState, '/game/state', self.on_game_state, 10)

        self.console.print(
            "[bold cyan]Analytics node ready.[/bold cyan]"
        )
        self.console.print(
            "Waiting for game to finish... "
            "or run: [cyan]ros2 run turtle_game analytics[/cyan] "
            "after the game to analyse the last session."
        )

        # Also run immediately if a log exists
        self.create_timer(2.0, self.auto_run)
        self._auto_ran = False

    def auto_run(self):
        if not self._auto_ran:
            self._auto_ran = True
            self.run_analytics()

    def on_game_state(self, msg):
        if msg.state == "GAME_OVER":
            # Small delay to let data_logger finish writing
            self.create_timer(1.5, self.run_analytics)

    def find_latest_summary(self):
        files = glob.glob(os.path.join(LOG_DIR, "*_summary.json"))
        if not files:
            return None
        return max(files, key=os.path.getmtime)

    def run_analytics(self):
        path = self.find_latest_summary()
        if not path:
            self.console.print(
                "[dim]No session log found yet. "
                "Play a game first.[/dim]")
            return

        with open(path) as f:
            data = json.load(f)

        self.console.print("\n")
        self.render_dashboard(data, path)

    def render_dashboard(self, data, path):
        events = data.get("events", [])
        poses = data.get("pose_samples", [])
        vels = data.get("vel_samples", [])
        coverage = data.get("grid_coverage", [])
        duration = data.get("duration_sec", 30)
        final_score = data.get("final_score", 0)

        catches = [e for e in events if e["type"] == "catch"]
        faults = [e for e in events if e["type"] == "fault"]
        bomb_hits = [e for e in catches
                     if e.get("turtle_type") == "bomb"]

        # ── header ────────────────────────────────────────────
        self.console.print(Panel(
            f"[bold cyan]Post-game analytics report[/bold cyan]\n"
            f"[dim]{os.path.basename(path)}[/dim]",
            box=rich_box.DOUBLE_EDGE
        ))

        # ── summary stats ─────────────────────────────────────
        stats = Table(box=rich_box.SIMPLE_HEAVY, expand=True,
                      show_header=False)
        stats.add_column(justify="left",  style="dim")
        stats.add_column(justify="right", style="bold white")
        stats.add_column(justify="left",  style="dim")
        stats.add_column(justify="right", style="bold white")

        avg_pts = (sum(e.get("points", 0) for e in catches)
                   / len(catches)) if catches else 0

        # Catch intervals for avg catch time
        catch_times = [e["t"] for e in catches]
        intervals = ([catch_times[i+1] - catch_times[i]
                      for i in range(len(catch_times)-1)]
                     if len(catch_times) > 1 else [])
        avg_interval = sum(intervals)/len(intervals) if intervals else 0

        stats.add_row(
            "Final score",   str(final_score),
            "Duration",      f"{duration:.1f}s"
        )
        stats.add_row(
            "Total catches", str(len(catches)),
            "Bomb hits",     str(len(bomb_hits))
        )
        stats.add_row(
            "Avg pts/catch", f"{avg_pts:.0f}",
            "Avg catch time", f"{avg_interval:.1f}s"
        )
        stats.add_row(
            "Faults logged", str(len(faults)),
            "Pose samples",  str(len(poses))
        )
        self.console.print(stats)

        # ── catch rate sparkline ──────────────────────────────
        self.console.print("\n[dim]Catch rate over time (each char = 1s)[/dim]")
        bins = defaultdict(int)
        for e in catches:
            bins[int(e["t"])] += 1
        rate_series = [bins.get(i, 0) for i in range(int(duration)+1)]
        self.console.print(
            f"[cyan]{sparkline(rate_series, width=60)}[/cyan]"
            f"  peak: [bold]{max(rate_series) if rate_series else 0}[/bold]/s"
        )

        # ── velocity analysis ─────────────────────────────────
        if vels:
            lin_vels = [abs(v["linear"]) for v in vels]
            ang_vels = [abs(v["angular"]) for v in vels]
            avg_lin = sum(lin_vels) / len(lin_vels)
            max_lin = max(lin_vels)
            avg_ang = sum(ang_vels) / len(ang_vels)

            self.console.print("\n[dim]Velocity profile[/dim]")
            vel_table = Table(box=rich_box.SIMPLE, show_header=False,
                              expand=True)
            vel_table.add_column(width=14)
            vel_table.add_column()
            vel_table.add_column(width=8, justify="right")

            vel_table.add_row(
                Text("Avg linear",  style="green"),
                Text(bar(avg_lin, 4.0, color="green"), style="green"),
                Text(f"{avg_lin:.2f}", style="green")
            )
            vel_table.add_row(
                Text("Max linear",  style="cyan"),
                Text(bar(max_lin, 4.0, color="cyan"), style="cyan"),
                Text(f"{max_lin:.2f}", style="cyan")
            )
            vel_table.add_row(
                Text("Avg angular", style="orange1"),
                Text(bar(avg_ang, 4.0, color="orange1"), style="orange1"),
                Text(f"{avg_ang:.2f}", style="orange1")
            )
            self.console.print(vel_table)

            # Linear velocity sparkline
            self.console.print("[dim]Linear velocity history:[/dim]")
            self.console.print(
                f"[green]{sparkline(lin_vels, width=60)}[/green]"
            )

        # ── arena heatmap (ASCII) ─────────────────────────────
        if poses:
            self.console.print("\n[dim]Arena coverage heatmap (11x11)[/dim]")
            grid_size = 11
            heat = [[0]*grid_size for _ in range(grid_size)]
            for p in poses:
                col = min(int(p["x"]), grid_size-1)
                row = min(int(p["y"]), grid_size-1)
                heat[row][col] += 1

            max_heat = max(max(r) for r in heat) or 1
            heat_chars = " ·░▒▓█"
            for row in reversed(heat):
                line = ""
                for val in row:
                    idx = int((val/max_heat)*(len(heat_chars)-1))
                    line += heat_chars[idx] * 2
                self.console.print(
                    f"[cyan]|[/cyan][blue]{line}[/blue][cyan]|[/cyan]")

        # ── occupancy grid coverage ───────────────────────────
        if coverage:
            self.console.print("\n[dim]Occupancy grid coverage over time[/dim]")
            pcts = [c["pct"] for c in coverage]
            self.console.print(
                f"[blue]{sparkline(pcts, width=60)}[/blue]"
                f"  final: [bold]{pcts[-1]:.1f}%[/bold] mapped"
            )

        # ── catch breakdown ───────────────────────────────────
        type_counts = defaultdict(int)
        type_points = defaultdict(int)
        for e in catches:
            t = e.get("turtle_type", "unknown")
            type_counts[t] += 1
            type_points[t] += e.get("points", 0)

        if type_counts:
            self.console.print("\n[dim]Catch breakdown by type[/dim]")
            bt = Table(box=rich_box.SIMPLE, expand=True)
            bt.add_column("Type",   style="cyan")
            bt.add_column("Count",  justify="right")
            bt.add_column("Points", justify="right")
            bt.add_column("Share",  justify="right")
            total_catches = len(catches)
            colors = {"normal": "white", "golden": "yellow",
                      "evasive": "cyan", "freeze": "blue", "bomb": "red"}
            for ttype, count in sorted(
                    type_counts.items(), key=lambda x: -x[1]):
                col = colors.get(ttype, "white")
                pct = count/total_catches*100 if total_catches else 0
                pts = type_points[ttype]
                bt.add_row(
                    Text(ttype, style=col),
                    str(count),
                    Text(str(pts), style="green" if pts > 0 else "red"),
                    f"{pct:.0f}%"
                )
            self.console.print(bt)

        # ── fault log ─────────────────────────────────────────
        if faults:
            self.console.print("\n[dim]Fault log[/dim]")
            for f in faults:
                self.console.print(
                    f"  [red][{f['t']:.1f}s][/red] {f['details']}")
        else:
            self.console.print(
                "\n[green]No faults detected during session.[/green]")

        self.console.print(
            f"\n[dim]Full log: {path}[/dim]\n")


def main(args=None):
    rclpy.init(args=args)
    node = Analytics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
