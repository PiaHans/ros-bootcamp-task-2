
# Multi-Robot SLAM and Autonomous Exploration

## 1. Overview

This project implements a multi-robot autonomous exploration system using two TurtleBot3 robots in a simulated Gazebo environment.

The system enables both robots to:

* Build their own local occupancy maps using LiDAR and SLAM.
* Navigate autonomously without keyboard teleoperation.
* Detect and navigate toward unexplored regions using frontier-based exploration.
* Register and merge the individual robot maps into a common global occupancy grid.
* Coordinate exploration through a shared merged map.

The complete system is launched using a single master launch file.

---

## 2. System Architecture

The overall pipeline is:

```text
             Robot 1                         Robot 2
                │                               │
          LiDAR + Odometry                LiDAR + Odometry
                │                               │
          GMapping SLAM                   GMapping SLAM
                │                               │
          Local Map 1                     Local Map 2
                │                               │
                └──────────┐     ┌──────────────┘
                           ▼     ▼
                         Map Merge
                           │
                           ▼
                    Global /map
                           │
                    Frontier Explorer
                           │
                         Nav2
                           │
                    Autonomous Motion
```

Each robot operates in its own ROS 2 namespace:

```text
/robot1
/robot2
```

---

## 3. Robot Configuration

Two TurtleBot3 robots are spawned at configurable initial positions.

Example:

```bash
robot1_x:=0.0
robot1_y:=0.5

robot2_x:=-3.0
robot2_y:=1.5
```

The initial positions can be passed as launch arguments.

---

## 4. ROS 2 Namespaces

Each robot has an independent namespace to avoid conflicts between topics, services, and nodes.

### Robot 1

```text
/robot1/scan
/robot1/odom
/robot1/map
/robot1/slam_gmapping
/robot1/explore_node
```

### Robot 2

```text
/robot2/scan
/robot2/odom
/robot2/map
/robot2/slam_gmapping
/robot2/explore_node
```

This allows both robots to run their SLAM and navigation pipelines independently.

---

## 5. SLAM Approach

GMapping is used for simultaneous localization and mapping.

Each robot uses:

* LiDAR scan data
* Odometry
* TF transformations

to construct its own occupancy grid.

The local maps are published as:

```text
/robot1/map
/robot2/map
```

The maps are continuously updated as the robots move through the environment.

---

## 6. Map Registration and Merging

The individual robot maps are combined using the `multirobot_map_merge` package.

The map merge node:

```text
/map_merge
```

receives the maps from both robots and produces a common merged occupancy grid:

```text
/map
```

The system is configured with:

```yaml
known_init_poses: false
```

so that the map merging system can perform automatic map registration rather than relying only on predefined initial map poses.

The merged map provides a common representation of the explored environment.

---

## 7. Frontier-Based Exploration

Autonomous exploration is implemented using a frontier-based exploration approach.

A **frontier** is the boundary between:

* known/free space, and
* unexplored/unknown space.

The exploration nodes identify suitable frontier targets and send navigation goals to Nav2.

The robots therefore decide where to explore based on the currently available unexplored regions rather than following a predefined path.

Exploration nodes:

```text
/robot1/explore_node
/robot2/explore_node
```

Frontier visualization is available through topics such as:

```text
/robot1/explore/frontiers
/robot2/explore/frontiers
```

---

## 8. Autonomous Navigation

Nav2 is used for autonomous navigation.

The navigation pipeline consists of:

```text
Frontier Target
      ↓
Nav2 Planner
      ↓
Controller
      ↓
Velocity Commands
      ↓
TurtleBot3
```

The robots continuously use the maps being generated during exploration for navigation and obstacle avoidance.

No keyboard teleoperation or hardcoded exploration route is used.

---

## 9. Exploration Termination

The exploration system continues selecting valid frontier targets while unexplored regions are available.

When the frontier explorer has no further valid reachable exploration targets, the robots stop exploring autonomously.

---

## 10. Master Launch File

The complete system is started using:

```bash
ros2 launch multirobot_map_merge master_launch.py \
  robot1_x:=0.0 \
  robot1_y:=0.5 \
  robot2_x:=-3.0 \
  robot2_y:=1.5
```

The master launch file starts:

* Gazebo simulation
* Robot 1
* Robot 2
* Robot state publishers
* LiDAR and odometry
* GMapping SLAM
* Nav2
* Map merging
* Frontier-based exploration

---

## 11. Building the Workspace

Clone/copy the submitted source into a ROS 2 workspace and build using:

```bash
cd ~/turtlebot3_ws

colcon build --symlink-install

source install/setup.bash
```

Then launch the complete system using the master launch command.

---

## 12. Verification

The following commands can be used to verify the running system.

### Check robot topics

```bash
ros2 topic list | grep robot1
ros2 topic list | grep robot2
```

### Check running nodes

```bash
ros2 node list | grep -E "explore|map_merge|robot1|robot2"
```

### Check local maps

```bash
ros2 topic echo /robot1/map --once
ros2 topic echo /robot2/map --once
```

### Check merged map

```bash
ros2 topic echo /map --once
```

### Check frontier information

```bash
ros2 topic echo /robot1/explore/frontiers --once
ros2 topic echo /robot2/explore/frontiers --once
```

### Check merged map publication

```bash
ros2 topic hz /map
```

---

## 13. Observed Results

During testing:

* Both TurtleBot3 robots spawned successfully in Gazebo.
* Both robots operated under independent ROS 2 namespaces.
* LiDAR and odometry data were available for both robots.
* GMapping generated individual occupancy maps.
* Frontier-based exploration generated exploration targets.
* Both robots moved autonomously without keyboard teleoperation.
* The individual maps were merged into a global `/map`.
* The merged map was continuously published during exploration.
* The robots eventually stopped after exploration targets were exhausted.

---

## 14. Key ROS Topics

| Purpose            | Robot 1                     | Robot 2                     |
| ------------------ | --------------------------- | --------------------------- |
| LiDAR              | `/robot1/scan`              | `/robot2/scan`              |
| Odometry           | `/robot1/odom`              | `/robot2/odom`              |
| Local Map          | `/robot1/map`               | `/robot2/map`               |
| Frontiers          | `/robot1/explore/frontiers` | `/robot2/explore/frontiers` |
| Exploration Status | `/robot1/explore/status`    | `/robot2/explore/status`    |

Merged map:

```text
/map
```

---

## 15. Main Components Used

* ROS 2
* Gazebo
* TurtleBot3
* GMapping
* Nav2
* `explore_lite`
* `multirobot_map_merge`
* LiDAR
* TF
* OccupancyGrid

---

## 16. Summary

This project demonstrates autonomous multi-robot exploration using two TurtleBot3 robots.

Each robot independently performs SLAM and navigation while frontier-based exploration determines unexplored regions. The resulting local maps are automatically registered and merged into a common global occupancy grid.

The entire system can be reproduced using the provided source code and a single master launch file.
