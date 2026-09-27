# Multi-Robot Autonomous Exploration and Mapping


### Solution approach
Each robot handle its own immediate driving and local mapping, while a lightweight coordination layer keeps them aligned.
Everything runs inside ROS 2 and Gazebo. Each robot operates in its own isolated namespace (`tb3_0` and `tb3_1`), so their laser scans, wheel movements, and path planners never clash. Above both robots sit two central components: one that stitches their individual maps into a single global floor plan, and another that coordinates where they should drive next so they do not collide

### SLAM and Exploration

Each robot runs its own SLAM to map what it sees with its lidar scanner. Because the robots keep their mapping separate, any brief sensor noise or slip on one robot doesn't corrupt the other's view.
To make them explore on their own, our explorer node constantly looks at the boundary between what is already mapped and what is still unknown. It groups these unexplored spots together, calculates which robot is closest and which area offers the most new ground to cover, and sends goal coordinates directly to each robot's Nav2 stack.

### Map Merging

As the robots drive around, they publish their local map views. The map merger node takes these partial grids and blends them together in a shared coordinate frame.
The blending logic is whenever an obstacle is seen by either robot, it gets drawn as a solid wall. Unexplored regions remain gray until at least one robot drives close enough to clear them into open space. The result is a continuously growing map that updates live as the robots discover new hallways and doorways.

### Running the Project

After setting up the workspace:

1. Build the workspace with `colcon build` and `source install/setup.bash`.
2. Start the simulation with the main launch file:
   `ros2 launch multi_bot_bringup multi_robot_slam.launch.py`

Once launched, Gazebo and RViz will open showing both robots actively scanning. You can watch the shared map expand as the robots explore different sections of the arena
