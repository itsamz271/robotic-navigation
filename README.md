# Project Description
This project involves unmanned navigation of a robot on the first floor of Prince Sultan University. It allows the user to send a goal location(s) to the robot and the robot will proceed to navigate accordingly. The project aimed to maximize productivity and lower operational costs when delivering documents from office to office.

This project allows the user to simulate that functionality using turtlebot3.

My Youtube Demonstration Video: https://www.youtube.com/watch?v=SAAhVrnk0KA&list=PL8coWtscjc1G1n_14U2CBn_1LRjkEF5g0&index=2&t=91s

# Files:
1. building101_gf.pgm and building101_gf.yaml: The map files of the university's first floor
2. building101.db: A local lightweight database which stores the locations
3. databaseControl.py: A script used to do CRUD operations to the database
4. building101_nav2.0.py: the executable file

# Setup:  
## Pre-requisites
1. Ubuntu 20.04  
2. ROS-Noetic: http://wiki.ros.org/noetic/Installation
3. Turtlebot3: https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/
4. Turtlebot3 Simulation Package: https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation
5. Turtlebot3 SLAM & Navigation Packages
6. Create a package and paste this project inside. Next, run `catkin_make`
7. install the sqlite3 package: `pip install db-sqlite3`

## Running the project
### Spawn turtlebot3 in an empty gazebo world
`roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch`
### Run RVIZ
`roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=path/to/project/building101_gf.yaml`
### Run the project script
Make sure the script is an executable  
`rosrun your_package_name building101_nav2.0.py`

