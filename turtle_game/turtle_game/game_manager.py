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
from std_msgs.msg import Empty
from turtle_game_interfaces.msg import GameState, TurtleType
from enum import Enum
from rich.console import Console
from rich.live import Live
from rich.table import Table
from rich.text import Text
from rich import box as rich_box


class State(Enum):
    IDLE = "IDLE"
    COUNTDOWN = "COUNTDOWN"
    RUNNING = "RUNNING"
    GAME_OVER = "GAME_OVER"


class GameManager(Node):
    def __init__(self):
        super().__init__('game_manager')
        self.console = Console()

        # Game state
        self.state = State.IDLE
        self.score = 0
        self.time_remaining = 30
        self.combo = 0
        self.combo_multiplier = 1.0
        self.last_catch_time = 0.0
        self.catches = []
        self.best_combo = 0

        # Publishers
        self.state_pub = self.create_publisher(GameState, '/game/state', 10)
        self.start_pub = self.create_publisher(Empty, '/game/start', 10)

        # Subscribers
        # Phase 5.4: subscribe to typed catch instead of generic empty
        self.create_subscription(
            TurtleType, '/game/catch_type', self.on_typed_catch, 10)
        self.create_subscription(Empty, '/game/start_request', self.on_start_request, 10)

        # 1 Hz game loop timer
        self.timer = self.create_timer(1.0, self.game_loop)

        # Rich live display
        self.live = Live(self.make_hud(), refresh_per_second=4, console=self.console)
        self.live.start()

        self.console.print(
            "[bold green]Game manager ready. Press S in the player terminal "
            "to start.[/bold green]")

        self.declare_parameter('difficulty', 'medium')
        self.declare_parameter('auto_start', False)
        diff = self.get_parameter('difficulty').get_parameter_value().string_value
        diff_settings = {
            'easy':   {'spawn_interval': 3.0, 'game_time': 45},
            'medium': {'spawn_interval': 2.0, 'game_time': 30},
            'hard':   {'spawn_interval': 1.0, 'game_time': 20},
        }
        settings = diff_settings.get(diff, diff_settings['medium'])
        self.total_time = settings['game_time']
        self.spawn_interval = settings['spawn_interval']

        if self.get_parameter('auto_start').value:
            self.auto_start_timer = self.create_timer(0.1, self.start_automatically)

    def start_automatically(self):
        """Start one autonomous game after node initialization."""
        self.auto_start_timer.cancel()
        self.on_start_request(Empty())

    def on_start_request(self, msg):
        if self.state == State.IDLE or self.state == State.GAME_OVER:
            self.state = State.COUNTDOWN
            self.score = 0
            self.time_remaining = self.total_time
            self.combo = 0
            self.combo_multiplier = 1.0
            self.last_catch_time = 0.0
            self.catches = []
            self.best_combo = 0
            self.countdown_value = 3
            self.console.print("[bold yellow]Get ready...[/bold yellow]")

    def on_typed_catch(self, msg):
        if self.state != State.RUNNING:
            return

        now = self.get_clock().now().nanoseconds / 1e9
        time_since_last = now - self.last_catch_time if self.last_catch_time else 999

        # bomb turtle — deduct points, reset combo, no multiplier
        if msg.turtle_type == "bomb":
            self.score = max(0, self.score + msg.points_value)
            self.combo = 0
            self.combo_multiplier = 1.0
            self.console.print("[bold red]BOMB! Points deducted. Combo reset.[/bold red]")
            self.last_catch_time = now
            return

        # freeze turtle — add time, still awards points
        if msg.turtle_type == "freeze":
            self.time_remaining = min(60, self.time_remaining + 5)
            self.console.print("[bold blue]+5 seconds! Freeze turtle caught.[/bold blue]")

        # combo logic for all non-bomb types
        if time_since_last < 5.0:
            self.combo += 1
            self.combo_multiplier = min(1.0 + self.combo * 0.5, 4.0)
        else:
            self.combo = 0
            self.combo_multiplier = 1.0

        # track best combo
        if self.combo > self.best_combo:
            self.best_combo = self.combo

        # speed bonus for catching within 3 seconds of last catch
        speed_bonus = 50 if time_since_last < 3.0 else 0
        points = int((msg.points_value + speed_bonus) * self.combo_multiplier)
        self.score += points
        self.last_catch_time = now
        self.catches.append((now, points, msg.turtle_type))

        # colored terminal feedback per type
        type_colors = {
            "normal":  "white",
            "golden":  "yellow",
            "evasive": "cyan",
            "freeze":  "blue",
        }
        color = type_colors.get(msg.turtle_type, "white")
        speed_tag = " [bold]SPEED BONUS![/bold]" if speed_bonus else ""
        self.console.print(
            f"[{color}]+{points} ({msg.turtle_type})[/{color}]"
            f"  combo x{self.combo}  ×{self.combo_multiplier:.1f}{speed_tag}"
        )

    def game_loop(self):
        if self.state == State.COUNTDOWN:
            if self.countdown_value > 0:
                self.console.print(f"[bold cyan]{self.countdown_value}...[/bold cyan]")
                self.countdown_value -= 1
            else:
                self.state = State.RUNNING
                self.last_catch_time = self.get_clock().now().nanoseconds / 1e9
                self.start_pub.publish(Empty())
                self.console.print("[bold green]GO![/bold green]")

        elif self.state == State.RUNNING:
            self.time_remaining -= 1
            if self.time_remaining <= 0:
                self.time_remaining = 0
                self.state = State.GAME_OVER
                self.show_results()

        self.publish_state()
        self.live.update(self.make_hud())

    def publish_state(self):
        msg = GameState()
        msg.state = self.state.value
        msg.score = self.score
        msg.time_remaining = self.time_remaining
        msg.combo = self.combo
        msg.combo_multiplier = self.combo_multiplier
        self.state_pub.publish(msg)

    def make_hud(self):
        table = Table(box=rich_box.ROUNDED, expand=True, show_header=False)
        table.add_column(justify="center")
        table.add_column(justify="center")
        table.add_column(justify="center")

        state_color = {
            State.IDLE:      "dim",
            State.COUNTDOWN: "yellow",
            State.RUNNING:   "green",
            State.GAME_OVER: "red"
        }.get(self.state, "white")

        time_style = "bold red" if self.time_remaining <= 10 else "bold yellow"

        table.add_row(
            Text(f"State: {self.state.value}", style=state_color),
            Text(f"Score: {self.score}", style="bold cyan"),
            Text(f"Time: {self.time_remaining}s", style=time_style)
        )
        table.add_row(
            Text(f"Combo: x{self.combo}", style="magenta"),
            Text(f"Multiplier: {self.combo_multiplier:.1f}x", style="bold magenta"),
            Text(f"Catches: {len(self.catches)}", style="green")
        )
        return table

    def show_results(self):
        self.live.stop()

        # breakdown by turtle type
        type_counts = {}
        type_points = {}
        for _, pts, ttype in self.catches:
            type_counts[ttype] = type_counts.get(ttype, 0) + 1
            type_points[ttype] = type_points.get(ttype, 0) + pts

        # main stats table
        stats_table = Table(
            title="[bold]Game Over — Final Stats[/bold]",
            box=rich_box.DOUBLE_EDGE
        )
        stats_table.add_column("Stat", style="cyan")
        stats_table.add_column("Value", style="bold white")
        stats_table.add_row("Final Score", str(self.score))
        stats_table.add_row("Turtles Caught", str(len(self.catches)))
        avg = sum(p for _, p, _ in self.catches) / len(self.catches) if self.catches else 0
        stats_table.add_row("Avg Points/Catch", f"{avg:.0f}")
        stats_table.add_row("Best Combo", f"x{self.best_combo}")
        self.console.print(stats_table)

        # breakdown table
        breakdown_table = Table(
            title="[bold]Breakdown by type[/bold]",
            box=rich_box.SIMPLE_HEAVY
        )
        breakdown_table.add_column("Type", style="cyan")
        breakdown_table.add_column("Caught", justify="right")
        breakdown_table.add_column("Points", justify="right")

        type_colors = {
            "normal": "white", "golden": "yellow",
            "evasive": "cyan", "freeze": "blue"
        }
        for ttype, count in type_counts.items():
            color = type_colors.get(ttype, "white")
            pts = type_points.get(ttype, 0)
            breakdown_table.add_row(
                f"[{color}]{ttype}[/{color}]",
                str(count),
                f"+{pts}"
            )
        self.console.print(breakdown_table)

        self.save_highscore()
        self.show_leaderboard()

        # restart live display for next game
        self.live = Live(self.make_hud(), refresh_per_second=4, console=self.console)
        self.live.start()
        self.console.print("[dim]Press S in player terminal to play again.[/dim]")

    def save_highscore(self):
        import json
        import os
        path = os.path.expanduser("~/.turtlenav_scores.json")
        scores = []
        if os.path.exists(path):
            with open(path) as f:
                scores = json.load(f)
        scores.append({
            "score": self.score,
            "catches": len(self.catches),
            "best_combo": self.best_combo
        })
        scores.sort(key=lambda x: x["score"], reverse=True)
        scores = scores[:10]
        with open(path, "w") as f:
            json.dump(scores, f, indent=2)
        self.console.print("[dim]Score saved to ~/.turtlenav_scores.json[/dim]")

    def show_leaderboard(self):
        import json
        import os
        path = os.path.expanduser("~/.turtlenav_scores.json")
        if not os.path.exists(path):
            self.console.print("[dim]No scores yet.[/dim]")
            return

        with open(path) as f:
            scores = json.load(f)

        lb_table = Table(
            title="[bold cyan]Leaderboard — Top 10[/bold cyan]",
            box=rich_box.SIMPLE_HEAVY
        )
        lb_table.add_column("Rank", style="dim", width=6)
        lb_table.add_column("Score", style="bold cyan")
        lb_table.add_column("Catches", style="green")
        lb_table.add_column("Best Combo", style="magenta")

        medals = {1: "[yellow]1st[/yellow]", 2: "[white]2nd[/white]", 3: "[orange1]3rd[/orange1]"}
        for i, s in enumerate(scores[:10], 1):
            rank = medals.get(i, f"{i}th")
            lb_table.add_row(
                rank,
                str(s["score"]),
                str(s.get("catches", "-")),
                f"x{s.get('best_combo', '-')}"
            )
        self.console.print(lb_table)

    def destroy_node(self):
        self.live.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GameManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
