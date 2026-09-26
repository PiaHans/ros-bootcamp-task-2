HOW I BUILT THE SOLUTION

I first started with spawning one turtlebot in an empty world, followed by trying tu use the world given to us in on spot round in takneek. once that was spawned properly and I verified that i could use keyboard teleops to operate it, I turned to making a map. first I used keyboard teleops to explore and map the maze. then proceeded to autonomous exploration. 
After achieving all this for one robot, i continued to extend it to two of them, and worked on map merging.

SLAM AND EXPLORATION APPROACH
Each robot uses its LIDAR and odom data to make a map using slam toolbox.
for navigation, the bots currently use nav2 to go to the center of the nearest frontier's boundary. 
also a failsafe mechanism exists to help in case it is stuck. if robot shows less than 5 cm movement in 7 seconds, it will move away from nearest obstacle.
I had initially not used nav2, but rather simply directed the bot to the nearest frontier's direction. however, the bot would get stuck, rotating in its own location, almost every time. 

MAP MERGING APPROACH
Current approach involves using KIP of bots and transforming the local map coordinates from each bot into a unified global coordinate system to get a unified map.
Before this, I tried scaling and aligning each bot's map into a common map based on matching features of both maps in real time. though this worked fine but took long to get the correct alignment and would sometimes never align.

HOW TO RUN IT

colcon build
source /opt/ros/humble/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch bringup main.launch.py

Some updates can still be made and I am working on them!
