# Autonomous Multi-Robot SLAM, Exploration & Map Merging

**Candidate ID:** 250419_harsh  
**Task:** Inter-IIT ROS Bootcamp - Task 2  
**Platform:** ROS 2 Humble | Gazebo 11 | TurtleBot3 Waffle Pi  

---

## 1. System Overview

This repository implements an autonomous multi-robot exploration, mapping, and map-merging system for two TurtleBot3 Waffle Pi robots operating simultaneously in an unknown environment. The robots autonomously discover frontiers, navigate through obstacle fields using Nav2, build local maps with SLAM Toolbox, and merge their maps into a unified global occupancy grid in real time.

```mermaid
graph TD
    subgraph Simulation
        GZ[Gazebo 11 Engine]
        R1_TB3[Robot 1: /robot1]
        R2_TB3[Robot 2: /robot2]
        GZ --> R1_TB3
        GZ --> R2_TB3
    end

    subgraph Robot 1 Stack [/robot1]
        R1_TF[TF Relay /robot1/tf]
        R1_SLAM[SLAM Toolbox: /robot1/map]
        R1_NAV[Nav2 Stack: /robot1/navigate_to_pose]
        R1_EXP[Frontier Explorer: Yamauchi]
        R1_TB3 --> R1_TF
        R1_TF --> R1_SLAM
        R1_SLAM --> R1_NAV
        R1_NAV --> R1_EXP
        R1_EXP --> R1_NAV
    end

    subgraph Robot 2 Stack [/robot2]
        R2_TF[TF Relay /robot2/tf]
        R2_SLAM[SLAM Toolbox: /robot2/map]
        R2_NAV[Nav2 Stack: /robot2/navigate_to_pose]
        R2_EXP[Frontier Explorer: Yamauchi]
        R2_TB3 --> R2_TF
        R2_TF --> R2_SLAM
        R2_SLAM --> R2_NAV
        R2_NAV --> R2_EXP
        R2_EXP --> R2_NAV
    end

    subgraph Real-Time Fusion
        MM[Map Merger Node]
        R1_SLAM -->|/robot1/map| MM
        R2_SLAM -->|/robot2/map| MM
        MM -->|/map| RVIZ[RViz2 Unified Display]
    end
```

---

## 2. Key Architecture & Features

### 2.1 Multi-Robot Namespacing & Isolation
- **Namespaces:** Each robot operates strictly inside its isolated namespace: `/robot1` and `/robot2`.
- **TF Tree Hierarchy:**
  - Robot 1: `robot1/map` $\rightarrow$ `robot1/odom` $\rightarrow$ `robot1/base_footprint` $\rightarrow$ `robot1/base_scan`
  - Robot 2: `robot2/map` $\rightarrow$ `robot2/odom` $\rightarrow$ `robot2/base_footprint` $\rightarrow$ `robot2/base_scan`
  - Merged Global Frame: `map` $\rightarrow$ `robot1/map` and `map` $\rightarrow$ `robot2/map` via static identity transforms.
- **TF Relay:** Solves ROS 2 Humble Nav2's hardcoded `/tf` remapping by bridging transforms between the global `/tf` topic and namespaced `/robot1/tf` & `/robot2/tf` topics with `TransientLocal` durability.

### 2.2 Autonomous Frontier Exploration (`exploration`)
- **Yamauchi Algorithm:** Identifies frontiers between free space (`0`) and unknown space (`-1`).
- **Obstacle Clearance Buffer:** Dilates obstacles by a $5 \times 5$ kernel ($10\text{ cm}$ safety margin) to eliminate frontier goals placed inside lethal costmap inflation zones.
- **Clustering:** Groups frontier cells into connected components using `scipy.ndimage.label`; filters out clusters smaller than 4 cells (noise rejection).
- **Goal Selection & Blacklisting:** Computes cluster centroids, selects the closest candidate to the robot pose, and maintains a spatial blacklist ($0.35\text{ m}$ radius) to permanently discard unreachable or aborted goals.
- **RViz Visualization:** Publishes real-time candidate frontier points (cyan) and targeted goals (red sphere) to `/<namespace>/frontier_markers`.

### 2.3 Real-Time Map Merging (`map_merger`)
- Subscribes to `/robot1/map` and `/robot2/map` with `TransientLocal` QoS.
- Dynamically computes the global bounding box encompassing both robots.
- Uses vectorized NumPy grid operations to perform cell fusion:
  $$\text{Merged}(x, y) = \begin{cases} \max(v_1, v_2) & \text{if both known} \\ v_1 & \text{if only } v_1 \text{ known} \\ v_2 & \text{if only } v_2 \text{ known} \\ -1 & \text{otherwise} \end{cases}$$
- Publishes the fused occupancy grid on `/map` at $1\text{ Hz}$.

---

## 3. Package Structure

```
src/
├── bringup/                  # Master launch, Nav2 configs & RViz
│   ├── config/               # nav2_r1.yaml, nav2_r2.yaml
│   ├── launch/               # master.launch.py, nav2_r1.launch.py, nav2_r2.launch.py
│   └── rviz/                 # multi_robot.rviz
├── exploration/              # Autonomous Yamauchi frontier explorer
│   ├── exploration/          # frontier_explorer.py
│   └── launch/               # explore.launch.py
├── map_merger/               # Real-time multi-robot map fusion
│   ├── map_merger/           # map_merger_node.py
│   └── launch/               # map_merge.launch.py
├── robot1_slam/              # SLAM Toolbox node & config for Robot 1
│   ├── config/               # slam_toolbox.yaml
│   └── launch/               # slam.launch.py
├── robot2_slam/              # SLAM Toolbox node & config for Robot 2
│   ├── config/               # slam_toolbox.yaml
│   └── launch/               # slam.launch.py
└── simulation/               # Dual TurtleBot3 Gazebo world spawner
    └── launch/               # simulation.launch.py
```

---

## 4. Installation & Build

### Prerequisites
- Ubuntu 22.04 LTS
- ROS 2 Humble Hawksbill
- Gazebo 11 (`gazebo_ros_pkgs`)
- Nav2 (`nav2_bringup`, `nav2_msgs`)
- SLAM Toolbox (`slam_toolbox`)
- Python 3 with `numpy`, `scipy`

### Build Workspace
```bash
cd ~/multi_robot_ws
colcon build --symlink-install
source install/setup.bash
```

---

## 5. Running the System

### Single Master Command
To launch the complete system (Gazebo simulation, dual SLAM, dual Nav2 stacks, map merger, dual frontier explorers, and RViz):

```bash
source ~/multi_robot_ws/install/setup.bash
ros2 launch bringup master.launch.py
```

### Running Modules Individually (for Debugging)
If testing individual components step-by-step:

1. **Simulation (Gazebo with dual robots):**
   ```bash
   ros2 launch simulation simulation.launch.py
   ```
2. **SLAM for Robot 1 & Robot 2:**
   ```bash
   ros2 launch robot1_slam slam.launch.py
   ros2 launch robot2_slam slam.launch.py
   ```
3. **Nav2 Navigation Stacks:**
   ```bash
   ros2 launch bringup nav2_r1.launch.py
   ros2 launch bringup nav2_r2.launch.py
   ```
4. **Map Merger:**
   ```bash
   ros2 launch map_merger map_merge.launch.py
   ```
5. **Frontier Exploration:**
   ```bash
   ros2 launch exploration explore.launch.py robot_namespace:=robot1
   ros2 launch exploration explore.launch.py robot_namespace:=robot2
   ```
6. **RViz Visualization:**
   ```bash
   rviz2 -d $(ros2 pkg prefix bringup)/share/bringup/rviz/multi_robot.rviz
   ```

---

## 6. Verification Checklist

| Requirement | Implementation Component | Status |
| :--- | :--- | :--- |
| **Namespace Isolation** | Separate `/robot1` and `/robot2` nodes, topics, actions, and TF frame prefixes | **PASS** |
| **Autonomous Exploration** | Yamauchi frontier exploration node (`frontier_explorer.py`), strictly no teleoperation | **PASS** |
| **Navigation Stack** | Nav2 DWBCritic + NavfnPlanner configured for each robot namespace | **PASS** |
| **SLAM Mapping** | Dual SLAM Toolbox instances generating `/robot1/map` and `/robot2/map` | **PASS** |
| **Map Fusion** | NumPy vectorized map merger publishing unified `/map` | **PASS** |
| **Single Master Launch** | `ros2 launch bringup master.launch.py` orchestrating entire pipeline | **PASS** |
