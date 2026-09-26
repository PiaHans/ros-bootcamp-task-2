# ROS Bootcamp Task 2: Multi-Robot Autonomous Exploration, SLAM & Map Merging

**Student Details:**
- **Name:** Utkarsh Gupta
- **Roll Number:** 251150
- **ROS Version:** ROS 2 Humble
- **Simulator:** Gazebo Classic 11
- **Platform:** Ubuntu 22.04 LTS (Jammy Jellyfish)
- **Demonstration Video:** Included in folder as 

---

## Project Overview

This project implements an autonomous multi-robot exploration, SLAM, and map merging system using two simulated TurtleBot3 Burger robots in Gazebo. The robots independently map an unknown arena, navigate without collisions using Nav2, discover and explore frontiers using the Yamauchi exploration algorithm, and continuously fuse their local maps into a unified global map.

---

## Package Architecture

The workspace contains 6 modular ROS 2 packages:

### 1. simulation
- Spawns two TurtleBot3 Burger robots inside Gazebo classic with isolated namespaces (`robot1` and `robot2`).
- Dynamically modifies model SDF frames to prefix `/base_scan`, `/odom`, `/base_footprint` with the robot namespace.
- Key files: `launch/simulation.launch.py`, `launch/spawn_robot.launch.py`.

### 2. robot_slam
- Runs independent, isolated `slam_toolbox` mapping nodes for both robots simultaneously.
- Frame trees: `robot1/map -> robot1/odom`, `robot2/map -> robot2/odom`.
- Topics: `/robot1/map`, `/robot2/map`.
- Key files: `config/slam_robot1.yaml`, `config/slam_robot2.yaml`, `launch/dual_slam.launch.py`.

### 3. navigation
- Full Nav2 navigation stack configured for dual-robot operation.
- Uses `RegulatedPurePursuitController` for local trajectory tracking and `NavfnPlanner` for global collision-free path planning.
- Costmaps: 3x3m local costmap with obstacle and inflation layers; global costmap tracking unknown space.
- Key files: `config/nav2_robot1.yaml`, `config/nav2_robot2.yaml`, `launch/dual_navigation.launch.py`.

### 4. exploration
- Autonomous frontier exploration based on Yamauchi's algorithm.
- Detects boundary cells where free space meets unknown space.
- Groups frontier cells into clusters via connected components, filters noise clusters, and routes robots to the closest reachable frontier using Nav2 `navigate_to_pose`.
- Concludes cleanly with `Exploration complete!` when all frontiers are explored.
- Key files: `exploration/frontier_explorer.py`, `launch/dual_exploration.launch.py`.

### 5. map_merger
- Fuses `/robot1/map` and `/robot2/map` into a unified global `/map` (`nav_msgs/OccupancyGrid`).
- Computes global bounding box, blends cell probabilities (obstacles take priority), and broadcasts dynamic TF: `map -> robot1/map` and `map -> robot2/map`.
- Key files: `map_merger/map_merger_node.py`, `launch/map_merger.launch.py`.

### 6. bringup
- Single-command master bringup for the entire multi-robot system with sequenced launch delays.
- Unified RViz visualization (`rviz/master.rviz`) displaying `/map`, `/robot1/map`, `/robot2/map`, laser scans, and TF tree.
- Key files: `launch/master.launch.py`, `rviz/master.rviz`.

---

## How to Run

### Method 1: Single Master Command (All-in-One)

```bash
cd ~/ros-bootcamp-task-2/251150_utkarsh
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
ros2 launch bringup master.launch.py
```

### Method 2: Step-by-Step Multi-Terminal Execution

1. **Terminal 1 — Simulation:**
   ```bash
   source install/setup.bash
   ros2 launch simulation simulation.launch.py
   ```
2. **Terminal 2 — Dual SLAM:**
   ```bash
   source install/setup.bash
   ros2 launch robot_slam dual_slam.launch.py
   ```
3. **Terminal 3 — Visualization:**
   ```bash
   source install/setup.bash
   rviz2 -d src/bringup/rviz/master.rviz
   ```
4. **Terminal 4 — Navigation:**
   ```bash
   source install/setup.bash
   ros2 launch navigation dual_navigation.launch.py
   ```
5. **Terminal 5 — Map Merger:**
   ```bash
   source install/setup.bash
   ros2 launch map_merger map_merger.launch.py
   ```
6. **Terminal 6 — Autonomous Exploration:**
   ```bash
   source install/setup.bash
   ros2 launch exploration dual_exploration.launch.py
   ```

---

## Verification Commands

- Active topics: `ros2 topic list`
- Action servers: `ros2 action list -t`
- Merged map info: `ros2 topic echo /map --no-arr --once`
- TF tree: `ros2 run tf2_tools view_frames`
