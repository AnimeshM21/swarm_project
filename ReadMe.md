# Swarm Robotics Project

A curiosity-driven project for building digital twins of a heterogeneous swarm of robots to simulate real-world industrial and defense applications. The goal is to convert multi-robot coordination theory into a realistic ROS 2 + Gazebo simulation.

## Overview

This project aims to build a fully functional Gazebo simulation implementing a swarm of heterogeneous robots, including:
- Ground Bots (differential drive)
- UAVs / Drones (quadrotor)
- Ground Bots with end-effectors *(planned)*

![3_bot_swarm](./images/multi_bot.png)

Currently, ground bots and drones are actively deployed through a **single unified launch file** that supports any mix of robot types. The setup replicates a **leaderless swarm** in a **communication-constrained environment**, where appropriate job scheduling algorithms select the respective type of bot based on the task at hand.

![1_drone](./images/bebop_drone.png)

This project utilises:
- **ROS 2** for communication
- **Gazebo Fortress (Ignition)** for simulation
- **ros_gz_bridge** for bidirectional ROS ↔ Gazebo topic bridging

## Prerequisites

- **ROS 2**: Humble
- **Gazebo**: Fortress (Ignition)
- **ros_gz**: ROS 2 ↔ Gazebo bridge

Ensure that ROS 2 Humble and Gazebo Fortress are correctly installed and sourced on your system.

## Installation

1. **Clone the repository**:
    ```bash
    git clone https://github.com/AnimeshM21/swarm_project.git
    cd swarm_project
    ```

2. **Build the workspace**:
    ```bash
    colcon build
    ```

3. **Source the setup file**:
    ```bash
    source install/setup.bash
    ```

## Usage

The entire project is driven by a **single unified launch file** — `swarm.launch.py` — and a **single teleop node** — `swarm_teleop`. Both support any combination of ground bots and drones via launch arguments.

### Launch the Swarm

```bash
# Default: 3 ground bots, 0 drones
ros2 launch swarm swarm.launch.py

# Custom mix
ros2 launch swarm swarm.launch.py num_bots:=2 num_drones:=1

# Drone only
ros2 launch swarm swarm.launch.py num_bots:=0 num_drones:=1

# Single ground bot
ros2 launch swarm swarm.launch.py num_bots:=1
```

The launch file automatically:
- Spawns all robots at collision-free random positions
- Generates and applies the correct ROS ↔ Gazebo bridge configuration
- Starts the `swarm_teleop` node pre-configured for the chosen fleet

### Teleop Controls

`swarm_teleop` is launched automatically. Controls are context-aware — they adapt to whether the active agent is a ground bot or a drone.

| Key | Ground Bot | Drone |
|-----|-----------|-------|
| `1`–`9` | Switch active agent | Switch active agent |
| `W` / `S` | Forward / Backward | Forward / Backward |
| `A` / `D` | Rotate Left / Right | Yaw Left / Right |
| `↑` / `↓` | Forward / Backward | Ascend / Descend |
| `←` / `→` | Rotate Left / Right | Strafe Left / Right |
| `SPACE` | Stop active agent | Hover |
| `0` | **Emergency stop ALL** | **Emergency stop ALL** |
| `X` | Quit | Quit |

## Project Structure

```
swarm_project/
└── src/swarm/
    ├── launch/
    │   └── swarm.launch.py       # Unified launch — all robot types & counts
    ├── model/                    # Modular robot description files (xacro)
    │   ├── robot.xacro           # Ground bot top-level
    │   ├── robot_drone.xacro     # Drone top-level
    │   ├── ground_bot.xacro      # Ground bot body
    │   ├── drone.xacro           # Drone body
    │   ├── ground_bot.gazebo     # Ground bot Gazebo plugins
    │   ├── drone.gazebo          # Drone Gazebo plugins
    │   ├── cameras.xacro         # Camera sensor macro
    │   ├── lidar.xacro           # LiDAR sensor macro
    │   ├── wheels.xacro          # Wheel geometry macro
    │   └── propellers.xacro      # Propeller geometry macro
    ├── swarm/
    │   └── swarm_teleop.py       # Unified teleop — ground bots + drones
    ├── world/                    # Gazebo world SDF files
    └── meshes/                   # 3D mesh assets (STL)
```

> **Note on `parameters/`:** Bridge parameters are now generated dynamically at launch time from `swarm.launch.py` and written to a temporary file. The static `bridge_params.yaml` has been removed.
