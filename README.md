# ROS-based-Autonomous-Navigation-with-Obstacle-Avoidance

This is the our project for course Artificial Intelligence(Course Code-CS210).
For this project we have to build a differential drive robot which can avoid obstacles. The robot design and other requirements can be found in the ppt added.

## Steps to get started
1. Clone this repository in your src.
2. Run <code> catkin_make </code>.
3. Run the command <code> roslaunch obstacle_avoidance spawn.launch </code> to launch gazebo, the bot and your world.
4. You can change the world by changing the name of the world file name in the spawn.launch file.
5. You can also change the position where the bot will spawn in the spawn.launch file.
6. Now open a second terminal and run the following command</br> <code>chmod +x ~/catkin_ws/src/obstacle_avoidance/scripts/start.py</code> or go into your scripts folder and then run the command</br><code>chmod +x start.py</code> .
8. Run the following command <code>rosrun obstacle_avoidance start.py</code> to start your obstacle avoidance algorithm.

## Prerequisites

* [ROS](http://wiki.ros.org/kinetic)  
* [Gazebo](http://wiki.ros.org/gazebo_ros_pkgs)

## Video
https://github.com/gracefullcoder/ROS-based-Autonomous-Navigation-with-Obstacle-Avoidance/assets/139680548/4876bf27-84b2-497a-a854-c36ac33b3da9

## Contributor
•	[Akshat Sahu](https://github.com/akshat2511)

•	[Nikhil Malgaya](https://github.com/nikhilmalgaya)

•	Sangam Birla

## To-Do / Improvements
* Use xacro to define the model to make it more readable and configurable
