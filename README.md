# TurtleBot-Localization
Localization and mapping based on particle filter with ROS and Gazebo

For now it is just localization here

Instructions to run this code: 

-- need to write --

If you already have everything installed (and created catkin_ws) run these commands:

cd ~/catkin_ws/src  
catkin_create_pkg localization nav_msgs sensor_msgs geometry_msgs rospy roscpp  
cd localization  
mkdir scripts  
**now copy files from this repository in scripts folder**  
cd ~/catkin_ws  
catkin_make  
  
Now, we have made catkin package and we only need to run everything. Now run these from new terminal:  
roslaunch turtlebot_gazebo turtlebot_world.launch   
**now delete objects that are not nedeed and put your map in gazebo, take a picture of it and make a map  
by turning obstacles into white on picture and everytning else in black, I will add a file later that helps do this**  
**also replace slam_map.png in scripts with your map, and change its dimensions and pixel_to_meter in particle_filter.py**  
  
In another terminal run:   
cd ~/catkin_ws  
source devel/bash.setup  
rosrun localization navigation.py  
  
In another terminal run:   
cd ~/catkin_ws  
source devel/bash.setup  
rosrun localization particle_filter.py  


