# Multi-Robot SLAM, Autonomous Exploration & Map Merging

A complete ROS 2 (Humble) system in which two simulated TurtleBot3 robots independently explore an unknown Gazebo maze arena, construct local occupancy-grid maps under isolated namespaces, and combine those maps into a single unified global occupancy grid in real time.

---

## Architecture Overview

```text
                           Gazebo Simulation (maze.world)
                                         │
                 ┌───────────────────────┴───────────────────────┐
                 ▼                                               ▼
         [Robot 1: /robot1]                              [Robot 2: /robot2]
       ├── Spawn: (-3.0, -3.0)                         ├── Spawn: (3.0, 3.0)
       ├── Sensors: /robot1/scan, /robot1/odom         ├── Sensors: /robot2/scan, /robot2/odom
       ├── SLAM: slam_toolbox                          ├── SLAM: slam_toolbox
       │   └── Local Map: /robot1/map                  │   └── Local Map: /robot2/map
       ├── Nav2: Path planning & collision avoidance   ├── Nav2: Path planning & collision avoidance
       └── Exploration: Frontier Explorer              └── Exploration: Frontier Explorer
                 │                                               │
                 └───────────────────────┬───────────────────────┘
                                         ▼
                                [Map Merger Node]
                                         │
                                         ▼
                            Global Unified Map (/map)
```

---

## Workspace Structure

The project strictly follows the modular structure specified in the task documentation:

```text
src/
├── simulation/      # Gazebo maze world model & parameterized robot spawning
├── robot1_slam/     # Robot 1 SLAM (slam_toolbox) & Nav2 navigation configuration
├── robot2_slam/     # Robot 2 SLAM (slam_toolbox) & Nav2 navigation configuration
├── exploration/     # Frontier detection, clustering, and goal dispatch node
├── map_merger/      # Real-time occupancy grid alignment and fusion node
└── bringup/         # Master launch orchestration and multi-robot RViz configuration
```

---

## Technical Approach

### 1. Robot Bring-up & Namespace Isolation
* Both robots are spawned from a single SDF template where odometry, base, and LiDAR frames are dynamically injected with the robot namespace (`robot1/base_scan`, `robot2/base_scan`).
* Coordinate frames and sensor topics are strictly isolated (`/robot1/...` and `/robot2/...`), preventing topic collisions.
* `robot_state_publisher` runs independently for each robot with a matching `frame_prefix`.

### 2. Independent SLAM Pipelines
* Each robot runs its own asynchronous instance of `slam_toolbox` (`async_slam_toolbox_node`).
* Robot 1 maps `robot1/map -> robot1/odom -> robot1/base_footprint`.
* Robot 2 maps `robot2/map -> robot2/odom -> robot2/base_footprint`.
* Both instances run Ceres-optimized scan matching and publish independent local occupancy grids to `/robot1/map` and `/robot2/map`.

### 3. Autonomous Frontier-Based Exploration
* Exploration is driven by an autonomous Python node based on the Yamauchi frontier exploration algorithm.
* **Frontier Detection:** Scans the occupancy grid to detect boundary cells where known free space (`value = 0`) borders unexplored space (`value = -1`).
* **Filtering & Clustering:** Uses morphological operations to discard cells adjacent to obstacles, clusters continuous frontier cells, and filters out noise clusters.
* **Smart Goal Selection:** Excludes points within 0.6m of the robot's current position and maintains a history of visited frontier locations so the robot continuously advances forward into unknown corridors.
* **Completion Detection:** When all reachable frontiers in an enclosed environment are mapped, the explorer cleanly logs mission completion and idles.

### 4. Nav2 Navigation & Obstacle Avoidance
* Each robot runs a dedicated Nav2 stack (controller, global planner, behavior server, and behavior tree navigator).
* Tuned DWB local planner and costmap inflation radius (0.25m) optimized for TurtleBot3 burger navigation through narrow corridors.
* Rotational speed is clamped to 0.6 rad/s to eliminate simulated wheel slip and maintain laser scan accuracy.

### 5. Map Alignment and Merging
* The `map_merger_node` subscribes to `/robot1/map` and `/robot2/map`.
* Maps are aligned into the global world reference frame `map` using the shared spatial geometry.
* Computes a dynamic global bounding box enclosing both maps.
* **Fusion Policy:** Fuses overlapping cells where obstacles (`100`) take highest priority over free space (`0`), and known cells overwrite unknown space (`-1`).
* Publishes the unified occupancy grid to `/map` using `transient_local` durability QoS.

---

## Prerequisites & Installation

### Environment
* **OS:** Ubuntu 22.04 LTS (Jammy Jellyfish)
* **ROS 2:** Humble Hawksbill
* **Simulator:** Gazebo Classic 11

### Dependencies
Install the required system packages:
```bash
sudo apt update
sudo apt install -y \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-turtlebot3 \
  ros-humble-turtlebot3-simulations \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  python3-colcon-common-extensions \
  ros-humble-tf-transformations \
  python3-transforms3d \
  python3-scipy
```

Ensure NumPy 1.x is installed for SciPy and ROS 2 C-bindings compatibility:
```bash
pip install "numpy<2"
```

Configure shell environment in `~/.bashrc`:
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
echo "export GAZEBO_IP=127.0.0.1" >> ~/.bashrc
echo "export GZ_TRANSPORT_LOCALHOST_ONLY=1" >> ~/.bashrc
source ~/.bashrc
```

---

## How to Build & Run

### 1. Build the Workspace
From the root of your workspace:
```bash
cd ~/ROS
colcon build --symlink-install
source install/setup.bash
```

### 2. Run the Complete System (Single Master Launch)
Launch the entire multi-robot simulation, SLAM, Nav2, exploration, map merger, and RViz with one command:
```bash
ros2 launch bringup master.launch.py
```

### 3. Parameterized Launch (Custom Spawn Coordinates)
You can supply custom initial coordinates at launch time:
```bash
ros2 launch bringup master.launch.py \
  robot1_x:=-3.0 robot1_y:=-3.0 robot1_yaw:=0.0 \
  robot2_x:=3.0 robot2_y:=3.0 robot2_yaw:=3.14159
```

### 4. Save the Merged Map (Optional)
To save the final stitched map to disk:
```bash
ros2 run nav2_map_server map_saver_cli -f ~/ROS/global_merged_map --ros-args -r map:=/map
```

---

## Submission Checklist
- [x] Simulation runs in Gazebo with two TurtleBot3 robots.
- [x] Robots isolated by namespace (`/robot1` and `/robot2`).
- [x] Independent SLAM instances generating localized occupancy grids.
- [x] Autonomous frontier-based exploration without teleoperation.
- [x] Nav2 collision avoidance and path planning active on both robots.
- [x] Real-time map registration and merging into unified `/map`.
- [x] Single master launch file (`master.launch.py`) with parameterized coordinates.
