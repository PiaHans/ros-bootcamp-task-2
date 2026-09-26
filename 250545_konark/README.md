# Multi-Robot SLAM, Autonomous Exploration & Map Merging

## How I Built the Solution
This project implements a multi-agent ROS 2 system where two TurtleBot3 robots autonomously explore an unknown Gazebo environment. The system is built using a completely decentralized namespace architecture:
- **Namespaces:** Each robot (`robot1` and `robot2`) is fully isolated. Their TF trees, sensor topics (LiDAR, Odometry), and ROS 2 nodes are prefixed.
- **URDF/Xacro:** The robot description is dynamically generated using `xacro` arguments to seamlessly inject the namespace into the `robot_state_publisher` and Gazebo plugins.
- **Robust Bringup:** A single master launch file (`multi_bot_spawn.launch.py`) orchestrates the Gazebo simulation, robot spawning, SLAM, Nav2 lifecycle managers, frontier exploration, and map merging.

## SLAM & Exploration Approach
- **SLAM:** Each robot runs an independent instance of `slam_toolbox` in asynchronous mode. They build local occupancy grids (`/robot1/map` and `/robot2/map`) using their respective `/scan` and `/odom` data.
- **Navigation:** A fully configured Nav2 stack runs for each robot, utilizing local and global costmaps properly scoped to their local SLAM frames.
- **Exploration:** We use `explore_lite` for frontier-based exploration. It analyzes the robot's local global costmap to detect frontiers (edges between known and unknown space) and autonomously issues `navigate_to_pose` Action commands to the Nav2 `bt_navigator` without any human teleoperation.

## Map Merging Approach
- **Algorithm:** We utilize `multirobot_map_merge` to combine the two local maps into a single global `/map`.
- **Handling Symmetrical Environments:** Highly periodic environments (like grid mazes) cause severe perceptual aliasing (false positive feature matches) for ORB/SURF algorithms. To counteract this, we hardened the map merger by setting a strict `estimation_confidence` threshold.
- **Structural Alignment:** We pass `map_start_pose` and `map_start_at_dock` to `slam_toolbox` so that each robot's local map is natively aligned to its exact physical Gazebo spawn coordinates (`[-2.0, 0]` and `[2.0, 0]`). The map merger then cleanly fuses these perfectly aligned grids side-by-side.

## How to Run It
1. Build the workspace:
   ```bash
   colcon build --symlink-install
   ```
2. Source the overlay:
   ```bash
   source install/setup.bash
   ```
3. Launch the complete master simulation:
   ```bash
   ros2 launch multi_agent_nav multi_bot_spawn.launch.py
   ```
4. Open RViz (with the provided config) to visualize the autonomous exploration and real-time map merging!
