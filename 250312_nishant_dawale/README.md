# Multi-Robot SLAM, Autonomous Exploration & Map Merging

**Student Roll Number:** 250312  
**Student Name:** Nishant Dawale  
**Course / Assignment:** ROS 2 Bootcamp Task 2  

---

## 1. System Architecture

The system is designed with a fully decoupled, modular architecture adhering to ROS 2 multi-robot standards:

```text
                           +----------------------------------------+
                           |              Gazebo Arena              |
                           |   (Outer walls, Divider with Gap, etc) |
                           +--------------------+-------------------+
                                                |
               +--------------------------------+--------------------------------+
               |                                                                 |
               v                                                                 v
+-----------------------------+                                   +-----------------------------+
|    Namespace: /robot1       |                                   |    Namespace: /robot2       |
|  - LiDAR: /robot1/scan      |                                   |  - LiDAR: /robot2/scan      |
|  - Odom: /robot1/odom       |                                   |  - Odom: /robot2/odom       |
|  - TF: robot1/base_footprint|                                   |  - TF: robot2/base_footprint|
|                             |                                   |                             |
|  [SLAM Toolbox Instance 1]  |                                   |  [SLAM Toolbox Instance 2]  |
|  --> Local Map: /robot1/map |                                   |  --> Local Map: /robot2/map |
|                             |                                   |                             |
|  [Nav2 Navigation Stack]    |                                   |  [Nav2 Navigation Stack]    |
|  - DWB Local Planner        |                                   |  - DWB Local Planner        |
|  - Costmaps & Navfn Planner |                                   |  - Costmaps & Navfn Planner |
|                             |                                   |                             |
|  [Frontier Explorer Node]   |                                   |  [Frontier Explorer Node]   |
|  - Detects unknown borders  |                                   |  - Detects unknown borders  |
|  - Regional bias (+y sector)|                                   |  - Regional bias (-y sector)|
+--------------+--------------+                                   +--------------+--------------+
               |                                                                 |
               | /robot1/map                                                     | /robot2/map
               +--------------------------------+--------------------------------+
                                                |
                                                v
                               +----------------------------------+
                               |         Map Merger Node          |
                               |  - ORB Feature Extraction        |
                               |  - RANSAC Rigid Registration     |
                               |  - Geometric Spawn Prior Fallback|
                               |  - Multi-Grid Occupancy Fusion   |
                               +----------------+-----------------+
                                                |
                                                v
                                 Published: /global_map
                                 (Unified Global Map)
```

---

## 2. How I Built the Solution

The solution is divided into six specialized ROS 2 packages inside `src/`:

1. **`simulation`**:
   - Spawns Gazebo with the competition maze world (`worlds/maze.world`), containing outer boundary walls, a central divider with a connecting gap, and internal obstacles.
   - Supports user-configurable spawn coordinates for both robots via launch arguments (`robot1_x`, `robot1_y`, `robot1_yaw`, `robot2_x`, `robot2_y`, `robot2_yaw`).
   - Automatically parses and generates isolated SDF models for each robot on the fly, assigning unique frame names (`robot1/odom`, `robot1/base_footprint`, `robot1/base_scan` and `robot2/odom`, `robot2/base_footprint`, `robot2/base_scan`).
   - Launches independent `robot_state_publisher` instances with frame prefixes.

2. **`robot1_slam` & `robot2_slam`**:
   - Provide independent configurations and launch files for `async_slam_toolbox_node`.
   - Keep TF trees completely separated (`robot1/map -> robot1/odom -> robot1/base_footprint` and `robot2/map -> robot2/odom -> robot2/base_footprint`).
   - Configure isolated Nav2 stacks with local costmaps, global costmaps, DWB trajectory planners, and recovery behaviors under `/robot1` and `/robot2`.

3. **`exploration`**:
   - Contains the autonomous frontier exploration node (`frontier_explorer.py`).
   - Employs morphological image processing to find frontiers between free and unknown space.
   - Evaluates frontiers using a utility function based on distance, cluster information gain, and regional divergence bias.
   - Commands Nav2 via the `NavigateToPose` action interface without requiring manual teleoperation.

4. **`map_merger`**:
   - Subscribes to `/robot1/map` and `/robot2/map`.
   - Employs a dual-mode registration pipeline: online ORB feature detection with RANSAC affine transformation estimation, backed by a geometric spawn coordinate prior fallback.
   - Fuses occupancy probabilities into a coherent `/global_map`.

5. **`bringup`**:
   - Orchestrates the full system in a single command using `master.launch.py`.
   - Starts simulation, robot spawning, SLAM, Nav2, explorers, map merger, and RViz2.

---

## 3. SLAM & Exploration Approach

### SLAM
- Each robot runs an isolated instance of `async_slam_toolbox_node` within its respective namespace.
- Laser scans (`/robot1/scan`, `/robot2/scan`) and wheel odometry are mapped into localized occupancy grids (`/robot1/map` and `/robot2/map`).
- Because frames are isolated, neither robot causes TF frame collisions.

### Frontier Detection Algorithm
1. **Occupancy Grid Segmentation**:
   - Unknown cells: $V = -1$
   - Free cells: $V = 0$
   - Occupied cells: $V \ge 50$
2. **Obstacle Inflation Mask**:
   - An obstacle safety inflation mask of radius $r_{\text{safe}} = 0.35\text{ m}$ is computed using morphological dilation. Frontiers inside this zone are discarded to ensure collision-free navigation targets.
3. **Frontier Extraction**:
   - Free space is dilated with a $3 \times 3$ kernel: $\mathcal{D}_{\text{free}} = \text{dilate}(\mathcal{M}_{\text{free}})$.
   - Frontiers are defined as the intersection of dilated free space and unknown space, minus the inflated obstacles:
     $$\mathcal{F} = (\mathcal{D}_{\text{free}} \cap \mathcal{M}_{\text{unknown}}) \setminus \mathcal{M}_{\text{obstacle\_inflated}}$$
4. **Clustering & Centroid Computation**:
   - Connected frontier pixels are clustered into contours using `cv2.findContours`.
   - Small contours below the minimum frontier threshold ($< 5\text{ cells}$) are discarded as sensor noise.
   - Centroids $(c_x, c_y)$ are computed using image moments ($M_{10}/M_{00}, M_{01}/M_{00}$) and mapped into metric world coordinates.

### Goal Selection & Regional Divergence
- To avoid redundant exploration or inter-robot conflicts:
  - **Robot 1** applies a positive bias towards frontiers in the $+y$ direction (upper half).
  - **Robot 2** applies a positive bias towards frontiers in the $-y$ direction (lower half).
- Utility Scoring Function:
  $$\text{Score} = \left(\frac{\sqrt{\text{Size}}}{d + 0.5}\right) \times \text{Penalty}_{\text{visited}} \times \text{Bonus}_{\text{region}}$$
  where $d$ is Euclidean distance from current robot pose, $\text{Penalty}_{\text{visited}}$ deprioritizes nearby already-explored sectors, and $\text{Bonus}_{\text{region}}$ enforces regional separation.
- **Deadlock Prevention**:
  - Failed or aborted navigation goals are recorded in a blacklist with an attempt counter. Unreachable pockets are suppressed after 3 failed attempts.
  - A timeout monitor replans if the robot remains engaged with an unprogressing goal for $> 45\text{ seconds}$.

### Clean Termination
- When all valid frontier clusters across the map have been cleared or are smaller than the noise threshold for 3 consecutive planning cycles, the explorer logs mission completion and cleanly halts movement.

---

## 4. Map Merging Approach

The map merger node continuously subscribes to `/robot1/map` and `/robot2/map` and generates `/global_map`.

### Dual-Mode Registration Pipeline
1. **Online Feature-Based Registration (ORB + RANSAC)**:
   - Occupancy values are mapped to an 8-bit grayscale image (Unknown: 127, Free: 255, Occupied: 0).
   - An ORB feature extractor identifies keypoints and computes 32-byte binary descriptors.
   - Descriptors are matched across maps using a Brute Force Matcher with Hamming distance and cross-checking.
   - Once sufficient overlapping geometry is discovered (e.g. at the central divider gap), rigid 2D transformation $[R \mid t]$ is estimated via `cv2.estimateAffinePartial2D` with RANSAC reprojection filtering.
   - Scale consistency is verified ($\approx 1.0$) to reject spurious symmetrical matches.
2. **Geometric Spawn Coordinate Prior Fallback**:
   - In early exploration when the robots are in isolated sectors of the arena with zero overlap, pure feature matching has no overlapping features.
   - The merger seamlessly falls back to the rigid transformation computed from the robots' initial spawn coordinates:
     $$T_{\text{prior}} = T(\mathbf{p}_1)^{-1} T(\mathbf{p}_2)$$
   - This guarantees that `/global_map` is valid, coherent, and actively stitching both maps from the very first second of the run.

### Probabilistic Occupancy Fusion
- A dynamic bounding box encompassing both Map 1 and the transformed Map 2 is created.
- The occupancy fusion follows deterministic priority rules:
  $$\mathcal{M}_{\text{global}}(x, y) = \begin{cases} 
  100 & \text{if } \mathcal{M}_1(x, y) = 100 \lor \mathcal{M}_2'(x, y) = 100 \\
  0 & \text{else if } \mathcal{M}_1(x, y) = 0 \lor \mathcal{M}_2'(x, y) = 0 \\
  -1 & \text{otherwise (unknown)}
  \end{cases}$$
- This guarantees that verified walls are never overwritten by free space, and explored free corridors are never overwritten by unobserved territory.

---

## 5. How to Run

### Prerequisites
- Ubuntu 22.04 LTS (Jammy) or 20.04 LTS (Focal)
- ROS 2 Humble Hawksbill (or Iron / Foxy)
- Gazebo (`gazebo_ros_pkgs`)
- Nav2 (`ros-humble-navigation2`, `ros-humble-nav2-bringup`)
- SLAM Toolbox (`ros-humble-slam-toolbox`)
- TurtleBot3 simulation packages (`turtlebot3`, `turtlebot3_gazebo`, `turtlebot3_description`)
- OpenCV (`python3-opencv`) and NumPy (`python3-numpy`)

```bash
sudo apt update
sudo apt install -y \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-turtlebot3* \
  ros-humble-gazebo-ros-pkgs \
  python3-opencv \
  python3-numpy
```

### Build Instructions
```bash
# Navigate to the workspace containing the src folder
cd /path/to/workspace

# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Build the packages
colcon build --symlink-install

# Source local workspace overlay
source install/setup.bash

# Set default TurtleBot3 model
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models
```

### Launch the Complete System
To launch the master orchestration file with default spawn positions:
```bash
ros2 launch bringup master.launch.py
```

### Custom Spawn Coordinates
You can supply custom initial positions and orientations:
```bash
ros2 launch bringup master.launch.py \
  robot1_x:=-2.5 \
  robot1_y:=-0.5 \
  robot1_yaw:=0.0 \
  robot2_x:=2.5 \
  robot2_y:=0.5 \
  robot2_yaw:=3.14159
```

### Custom World
To run with a custom world file:
```bash
ros2 launch bringup master.launch.py \
  world:=/path/to/custom_world.world
```

### Verify Output Topics
In another terminal, check published topics:
```bash
# Check the merged global map
ros2 topic hz /global_map
ros2 topic info /global_map

# Check individual maps
ros2 topic hz /robot1/map
ros2 topic hz /robot2/map

# Monitor navigation goals
ros2 topic echo /robot1/navigate_to_pose/_action/status
```

---

## 6. Demonstration Video

A video demonstration recording the complete multi-robot exploration, SLAM mapping, and map merging pipeline is provided as `Demonstration.mp4` / `Demonstration.webm` in this folder.
