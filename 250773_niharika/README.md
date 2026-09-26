
# Multi-Robot SLAM and Autonomous Exploration

## 1. Overview

This project implements a multi-robot autonomous exploration system using two TurtleBot3 robots in a Gazebo environment.

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
