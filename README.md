# TurtleNav

TurtleNav is an extensible `turtlesim` game that demonstrates ROS 2 topics,
services, control, filtering, mapping, health monitoring, and post-game analytics.

## Architecture

```text
GameManager -> /game/state -> Spawner -> /turtles_data -> Player or Chaser
                                      |                    |
                                      +-- kill_turtle <----+
Player pose -> Kalman Filter -> Player Controller
Player pose/catches -> Mapper -> RViz topics
All runtime topics -> Watchdog and Data Logger -> Analytics
```

The package provides ten executable nodes: `turtle_spawner`,
`target_chaser`, `game_manager`, `player_controller`, `pid_controller`,
`kalman_filter_node`, `occupancy_grid_mapper`, `watchdog`, `data_logger`,
and `analytics`.

`target_chaser` is an alternative autonomous controller. Do not run it beside
the manual player or PID controller because each publishes `/turtle1/cmd_vel`.

## Prerequisites

- Ubuntu with ROS 2 Humble, Iron, or Jazzy installed and sourced
- ROS 2 packages: `turtlesim`, `rviz2`, `geometry_msgs`, `nav_msgs`,
  `visualization_msgs`, `std_msgs`, and `std_srvs`
- Build and dependency tools: `colcon`, `rosdep`, CMake, and the ROS Python
  build tooling
- Python libraries: `numpy`, `rich`, and `pytest`
- `xterm` for the separate game-manager, player-controller, and watchdog
  terminals used by the default manual launch

For ROS 2 Humble on Ubuntu, install the system dependencies with:

```bash
sudo apt update
sudo apt install \
  ros-humble-turtlesim \
  ros-humble-rviz2 \
  ros-humble-geometry-msgs \
  ros-humble-nav-msgs \
  ros-humble-visualization-msgs \
  ros-humble-std-msgs \
  ros-humble-std-srvs \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-numpy \
  python3-pytest \
  python3-rich \
  xterm
```

Replace `humble` with your installed ROS 2 distribution where applicable.
Initialize rosdep once on a new machine, then resolve dependencies from the
workspace root:

```bash
sudo rosdep init
rosdep update
```

```bash
cd ~/turtle_game_ws/src/ros2_turtle_chaser
rosdep install --from-paths . --ignore-src -r -y
```

## Build

```bash
cd ~/turtle_game_ws
colcon build --symlink-install
source install/setup.bash
```

## Run

### Default game: manual player control

```bash
ros2 launch turtle_game turtle_game.launch.py
```

Press uppercase `S` in the player-controller terminal to start. Use `W/A/S/D`
to drive and Space to stop. The PID node is launched disabled; enable it only
when manual control is not publishing commands:

```bash
ros2 param set /pid_controller enabled true
```

### Autonomous chaser mode

```bash
ros2 launch turtle_game autonomous_turtle_game.launch.py
```

This profile auto-starts the game and runs `target_chaser`, without the player
or PID controller. Its linear speed is capped at 4.0 turtlesim units per
second.

### Analytics

`analytics` is intentionally on-demand rather than part of either launch:

```bash
ros2 run turtle_game analytics
```

It reads the newest session summary from `~/.turtlenav_logs`.

## RViz2

Live mapping is one of the core game features. Start RViz2 in a second
terminal while either game mode is running:

```bash
source ~/turtle_game_ws/install/setup.bash
rviz2
```

Set **Fixed Frame** to `map`, then add these live displays:

- `/map/occupancy_grid` as **Map**
- `/map/path` as **Marker**
- `/map/markers` as **MarkerArray**

The occupancy grid shows explored, danger, and high-value areas; the path
marker traces the player; and the marker array records golden and bomb catches.
You can also load the packaged configuration from
`share/turtle_game/rviz/rviz_Settings.rviz` after installation.

## Runtime behaviour

- **Spawner** creates normal, golden, evasive, bomb, and freeze turtles.
- **Game manager** handles countdowns, scoring, combos, and the game clock.
- **Kalman filter** produces `/turtles_data_filtered` for the manual player.
- **Mapper** records exploration, routes, danger zones, and high-value zones.
- **Watchdog** reports stale node heartbeats to `/system/fault`.
- **Data logger** persists JSONL events and JSON summaries outside the repo.

## Test

```bash
cd ~/turtle_game_ws
colcon test --packages-select turtle_game turtle_game_interfaces
colcon test-result --verbose
```
