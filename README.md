<!-- BlackRoad SEO Enhanced -->

# ulackroad motion planner

> Part of **[BlackRoad OS](https://blackroad.io)** — Sovereign Computing for Everyone

[![BlackRoad OS](https://img.shields.io/badge/BlackRoad-OS-ff1d6c?style=for-the-badge)](https://blackroad.io)
[![BlackRoad OS](https://img.shields.io/badge/Org-BlackRoad-OS-2979ff?style=for-the-badge)](https://github.com/BlackRoad-OS)
[![License](https://img.shields.io/badge/License-Proprietary-f5a623?style=for-the-badge)](LICENSE)

**ulackroad motion planner** is part of the **BlackRoad OS** ecosystem — a sovereign, distributed operating system built on edge computing, local AI, and mesh networking by **BlackRoad OS, Inc.**

## About BlackRoad OS

BlackRoad OS is a sovereign computing platform that runs AI locally on your own hardware. No cloud dependencies. No API keys. No surveillance. Built by [BlackRoad OS, Inc.](https://github.com/BlackRoad-OS-Inc), a Delaware C-Corp founded in 2025.

### Key Features
- **Local AI** — Run LLMs on Raspberry Pi, Hailo-8, and commodity hardware
- **Mesh Networking** — WireGuard VPN, NATS pub/sub, peer-to-peer communication
- **Edge Computing** — 52 TOPS of AI acceleration across a Pi fleet
- **Self-Hosted Everything** — Git, DNS, storage, CI/CD, chat — all sovereign
- **Zero Cloud Dependencies** — Your data stays on your hardware

### The BlackRoad Ecosystem
| Organization | Focus |
|---|---|
| [BlackRoad OS](https://github.com/BlackRoad-OS) | Core platform and applications |
| [BlackRoad OS, Inc.](https://github.com/BlackRoad-OS-Inc) | Corporate and enterprise |
| [BlackRoad AI](https://github.com/BlackRoad-AI) | Artificial intelligence and ML |
| [BlackRoad Hardware](https://github.com/BlackRoad-Hardware) | Edge hardware and IoT |
| [BlackRoad Security](https://github.com/BlackRoad-Security) | Cybersecurity and auditing |
| [BlackRoad Quantum](https://github.com/BlackRoad-Quantum) | Quantum computing research |
| [BlackRoad Agents](https://github.com/BlackRoad-Agents) | Autonomous AI agents |
| [BlackRoad Network](https://github.com/BlackRoad-Network) | Mesh and distributed networking |
| [BlackRoad Education](https://github.com/BlackRoad-Education) | Learning and tutoring platforms |
| [BlackRoad Labs](https://github.com/BlackRoad-Labs) | Research and experiments |
| [BlackRoad Cloud](https://github.com/BlackRoad-Cloud) | Self-hosted cloud infrastructure |
| [BlackRoad Forge](https://github.com/BlackRoad-Forge) | Developer tools and utilities |

### Links
- **Website**: [blackroad.io](https://blackroad.io)
- **Documentation**: [docs.blackroad.io](https://docs.blackroad.io)
- **Chat**: [chat.blackroad.io](https://chat.blackroad.io)
- **Search**: [search.blackroad.io](https://search.blackroad.io)

---


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
