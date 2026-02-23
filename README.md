# blackroad-motion-planner

> Robot/drone motion planning with A\* pathfinding, waypoint management, and collision detection.

## Features

- **A\* pathfinding** on a continuous grid with configurable resolution
- **Douglas-Peucker smoothing** to reduce path complexity
- **Collision detection** with clearance margin checking
- **Obstacle management** (static, dynamic, no-fly zones)
- **ASCII map visualization** of planning space
- **SQLite persistence** for waypoints, obstacles, and missions

## Quick Start

```bash
pip install -e .

# Add waypoints
python src/motion_planner.py add-waypoint depot 0 0
python src/motion_planner.py add-waypoint target 25 30 --speed 2.0

# Add obstacles
python src/motion_planner.py add-obstacle 10 15 3.0 --kind static

# Plan a path
python src/motion_planner.py plan depot target

# Check for collisions
python src/motion_planner.py check-collision <mission_id>

# Visualize the space
python src/motion_planner.py visualize --mission-id <mission_id>

# List all missions
python src/motion_planner.py list-missions
```

## CLI Reference

| Command | Description |
|---------|-------------|
| `add-waypoint NAME X Y [--z] [--speed] [--hover]` | Register a named waypoint |
| `add-obstacle CX CY RADIUS [--height] [--kind]` | Add an obstacle |
| `plan START GOAL` | Compute A\* path between two waypoints |
| `check-collision MISSION_ID [--clearance]` | Check path safety |
| `list-waypoints` | Show all waypoints |
| `list-missions` | Show all planned missions |
| `visualize [--mission-id]` | ASCII map of planning space |

## Algorithms

- **A\*** — octile heuristic, 8-directional movement, grid resolution configurable (default 0.5m)
- **Douglas-Peucker** — simplifies raw A\* output; default ε=0.4m
- **Collision Check** — iterates path points against all obstacle radii + clearance margin

## Development

```bash
pip install pytest pytest-cov flake8
pytest tests/ -v --cov=src
```
