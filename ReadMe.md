# Swarm Robotics Project

A curiosity-driven project for building digital twins of a heterogeneous swarm of robots to simulate real-world industrial and defense applications. The goal is to convert multi-robot coordination theory into a realistic ROS 2 + Gazebo simulation.

## Overview

This project aims to build a fully functional Gazebo simulation implementing a swarm of heterogeneous robots, including:
- Ground Bots
- Ground Bots with end-effectors
- UAVs

Currently, only the Ground Bot is deployed, and the communication architecture is being developed before integrating the other two robot types. The setup will replicate a **leaderless swarm** in a **communication-constrained environment**, where appropriate job scheduling algorithms select the respective type of bot from the three based on the type of task.

![5_bot_swarm](./images/multi_bot.png)



This project utilizes:
- **ROS 2** for communication
- **Gazebo** for simulation

## Prerequisites

- **ROS 2**: Humble
- **Gazebo**: Fortress
- **ros_gz**: ROS 2 <-> Gazebo bridge

Ensure that ROS 2 Humble and Gazebo Fortress are correctly installed and sourced on your system.

## Installation

1.  **Clone the repository**:
    ```bash
    git clone <repository_url>
    cd swarm_project
    ```

2.  **Build the workspace**:
    ```bash
    colcon build
    ```

3.  **Source the setup file**:
    ```bash
    source install/setup.bash
    ```

## Usage

This package provides several launch files to simulate different robot configurations in Gazebo.

### 1. Launch a Drone
Spawns a single drone in the simulation.
```bash
ros2 launch swarm drone.launch.py
```

### 2. Launch a Single Ground Bot
Spawns a single ground robot.
```bash
ros2 launch swarm single_bot.launch.py
```

### 3. Launch a Swarm (Multi-Bot)
Spawns multiple ground robots (default is 3) in a swarm configuration with random valid positions.
```bash
ros2 launch swarm swarm_bot.launch.py
```

## Project Structure

- **launch/**: Contains Python launch files for different robot configurations as mentioned above.
- **model/**: Robot description files, which have been made modular based on each aspect of the bot.
- **world/**: Gazebo world files (SDF), currently only an empty world, actively adding obstacles and relevant features.
- **parameters/**: Configuration files (e.g., bridge parameters).
- **meshes/**: 3D models for robots, primarily STL files imported from Fuel.


