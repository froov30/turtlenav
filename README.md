# TurtleNav

An extensible ROS 2 game demonstrating core robotics concepts: topics, services, control, sensor filtering, and health monitoring on the `turtlesim` platform.

## What this is

TurtleNav teaches robotics fundamentals by building a complete game loop around the turtlesim simulator. A player controls a turtle to catch moving targets while the system tracks game state, filters noisy sensor input, maps the arena, monitors node health, and logs all events. The architecture uses ROS 2 services, topics, and parameters to decouple game logic, control, and analytics.

## Quick start

### Prerequisites

- Ubuntu 22.04 with ROS 2 Humble (or Iron / Jazzy)
- `rosdep`, `colcon`, Python 3, numpy, rich, pytest

### Setup

```bash
sudo rosdep init && rosdep update
cd ~/turtle_game_ws
rosdep install --from-paths . --ignore-src -r -y
colcon build --symlink-install --packages-select turtle_game turtle_game_interfaces
source install/setup.bash
```

### Run

**Manual player mode (default):**
```bash
ros2 launch turtle_game turtle_game.launch.py
# Press uppercase 'S' to start, use W/A/S/D to drive, Space to stop
```

**Autonomous chaser mode:**
```bash
ros2 launch turtle_game autonomous_turtle_game.launch.py
```

**View analytics:**
```bash
ros2 run turtle_game analytics
```

### Monitor in RViz2

```bash
rviz2
```

Set Fixed Frame to `map`, then add these displays:
- `/map/occupancy_grid` as **Map**
- `/map/path` as **Marker**
- `/map/markers` as **MarkerArray**

## Architecture

| Component | Role |
|---|---|
| **game_manager** | Countdown, scoring, combos, game clock |
| **turtle_spawner** | Create and manage normal, golden, evasive, bomb, and freeze turtles |
| **player_controller** | Manual keyboard input handler |
| **pid_controller** | Velocity PID regulator for smooth motion |
| **target_chaser** | Autonomous turtle-tracking algorithm |
| **kalman_filter_node** | Denoise player pose from `/turtle1/pose` |
| **occupancy_grid_mapper** | Record exploration, danger zones, high-value zones |
| **watchdog** | Monitor node heartbeats, report faults |
| **data_logger** | Persist JSONL events and JSON session summaries |
| **analytics** | Post-game reporting (catches, arena map, timing) |

## Key features

- **Decoupled architecture**: game logic, control, and analytics communicate only through ROS topics and services
- **Real-time state visualization**: live occupancy grid and path tracing in RViz2
- **Sensor filtering**: Kalman filter cleans pose estimates before control
- **Health monitoring**: watchdog detects stale nodes and missed heartbeats
- **Deterministic testing**: full game session can be replayed and analyzed

## Test

```bash
colcon test --packages-select turtle_game turtle_game_interfaces
colcon test-result --verbose
```

## License

MIT License. See [LICENSE](LICENSE).

## Contact

GitHub: [@froov30](https://github.com/froov30)
