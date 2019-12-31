# RoboND-Term1-P4-Map-My-World
Project 4 of Udacity Robotics Software Engineer Nanodegree Program
![Overview](/screenshots/Overview.png)  

## Overview  
In this project you will create a 2D occupancy grid and 3D octomap from a simulated environment using your own robot with the RTAB-Map package.  
RTAB-Map (Real-Time Appearance-Based Mapping) is a popular solution for SLAM to develop robots that can map environments in 3D. RTAB-Map has good speed and memory management, and it provides custom developed tools for information analysis. Most importantly, the quality of the documentation on ROS Wiki (http://wiki.ros.org/rtabmap_ros) is very high. Being able to leverage RTAB-Map with your own robots will lead to a solid foundation for mapping and localization well beyond this Nanodegree program.  
For this project we will be using the `rtabmap_ros` package, which is a ROS wrapper (API) for interacting with RTAB-Map. Keep this in mind when looking at the relative documentation.  
* You will develop your own package to interface with the rtabmap_ros package.  
* You will build upon your localization project to make the necessary changes to interface the robot with RTAB-Map. An example of this is the addition of an RGB-D camera.  
* You will ensure that all files are in the appropriate places, all links are properly connected, naming is properly setup and topics are correctly mapped. Furthermore you will need to generate the appropriate launch files to launch the robot and map its surrounding environment.  
* When your robot is launched you will teleop around the room to generate a proper map of the environment.  

## Prerequisites/Dependencies  
* Gazebo >= 7.0  
* ROS Kinetic  
* ROS navigation package  
```
sudo apt-get install ros-kinetic-navigation
```
* ROS map_server package  
```
sudo apt-get install ros-kinetic-map-server
```
* ROS move_base package  
```
sudo apt-get install ros-kinetic-move-base
```
* ROS amcl package  
```
sudo apt-get install ros-kinetic-amcl
```
* ROS rtabmap-ros package
```
sudo apt-get install ros-kinetic-rtabmap-ros
```
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Setup Instructions (abbreviated)  
1. Meet the `Prerequisites/Dependencies`  
2. Open Ubuntu Bash and clone the project repository  
3. On the command line execute  
```bash
sudo apt-get update && sudo apt-get upgrade -y
```
4. Build and run your code.  

## Project Description  
Directory Structure  
```
.Map-My-World                                  # Map My World Project
├── catkin_ws                                  # Catkin workspace
│   ├── frames.pdf                             # view_frames result
│   ├── src
│   │   ├── ball_chaser                        # ball_chaser package        
│   │   │   ├── launch                         # launch folder for launch files
│   │   │   │   ├── ball_chaser.launch
│   │   │   ├── src                            # source folder for C++ scripts
│   │   │   │   ├── drive_bot.cpp
│   │   │   │   ├── process_images.cpp
│   │   │   ├── srv                            # service folder for ROS services
│   │   │   │   ├── DriveToTarget.srv
│   │   │   ├── CMakeLists.txt                 # compiler instructions
│   │   │   ├── package.xml                    # package info
│   │   ├── my_gokart                          # my_gokart package        
│   │   │   ├── launch                         # launch folder for launch files   
│   │   │   │   ├── gokart_description.launch
│   │   │   │   ├── world.launch
│   │   │   ├── urdf                           # urdf folder for xarco files
│   │   │   │   ├── my_gokart.gazebo
│   │   │   │   ├── my_gokart.xacro
│   │   │   ├── worlds                         # world folder for world files
│   │   │   │   ├── empty.world
│   │   │   │   ├── myoffice.world
│   │   │   ├── CMakeLists.txt                 # compiler instructions
│   │   │   ├── package.xml                    # package info
│   │   ├── my_robot                           # my_robot package        
│   │   │   ├── config                         # config folder for configuration files   
│   │   │   │   ├── base_local_planner_params.yaml
│   │   │   │   ├── costmap_common_params.yaml
│   │   │   │   ├── global_costmap_params.yaml
│   │   │   │   ├── local_costmap_params.yaml
│   │   │   ├── launch                         # launch folder for launch files   
│   │   │   │   ├── amcl.launch
│   │   │   │   ├── robot_description.launch
│   │   │   │   ├── world.launch
│   │   │   │   ├── mapping.launch
│   │   │   │   ├── localization.launch
│   │   │   ├── maps                           # maps folder for maps
│   │   │   │   ├── myoffice.pgm
│   │   │   │   ├── myoffice.yaml
│   │   │   ├── meshes                         # meshes folder for sensors
│   │   │   │   ├── hokuyo.dae
│   │   │   ├── rviz                           # rviz folder for rviz configuration files
│   │   │   │   ├── default.rviz
│   │   │   ├── urdf                           # urdf folder for xarco files
│   │   │   │   ├── my_robot.gazebo
│   │   │   │   ├── my_robot.xacro
│   │   │   ├── worlds                         # world folder for world files
│   │   │   │   ├── empty.world
│   │   │   │   ├── myoffice.world
│   │   │   ├── CMakeLists.txt                 # compiler instructions
│   │   │   ├── package.xml                    # package info
│   │   ├── pgm_map_creator                    # pgm_map_creator        
│   │   │   ├── launch                         # launch folder for launch files   
│   │   │   │   ├── request_publisher.launch
│   │   │   ├── maps                           # maps folder for generated maps
│   │   │   │   ├── Backup_map.pgm
│   │   │   │   ├── map.pgm
│   │   │   ├── msgs                           # msgs folder for communication files
│   │   │   │   ├── CMakeLists.txt
│   │   │   │   ├── collision_map_request.proto
│   │   │   ├── src                            # src folder for main function
│   │   │   │   ├── collision_map_creator.cc
│   │   │   │   ├── request_publisher.cc
│   │   │   ├── world                          # world folder for world files
│   │   │   │   ├── myoffice.world
│   │   │   │   ├── udacity_mtv
│   │   │   ├── CMakeLists.txt                 # compiler instructions
│   │   │   ├── LICENSE                        # License for repository
│   │   │   ├── README.md                      # README for documentation
│   │   │   ├── package.xml                    # package info
│   │   ├── teleop_twist_keyboard              # teleop_twist_keyboard
│   │   │   ├── CHANGELOG.rst                  # change log
│   │   │   ├── CMakeLists.txt                 # compiler instructions
│   │   │   ├── README.md                      # README for documentation
│   │   │   ├── package.xml                    # package info
│   │   │   ├── teleop_twist_keyboard.py       # keyboard controller
├── my_ball                                    # Model files 
│   ├── model.config
│   ├── model.sdf
├── screenshots                                # Screenshots
│   ├── Overview.png                           # Screenshot for overview
```
- [Overview.png](/screenshots/Overview.png): An overview screenshot to demo the final result
- [drive_bot.cpp](/catkin_ws/src/ball_chaser/src/drive_bot.cpp): ROS service C++ script, command the robot with specify speeds.  
- [process_images.cpp](/catkin_ws/src/ball_chaser/src/process_images.cpp): ROS service C++ script, process the camera image and return requested speeds.  
- [gokart_description.launch](/catkin_ws/src/my_gokart/launch/gokart_description.launch): Create gokart model in Gazebo world.  
- [world.launch](/catkin_ws/src/my_gokart/launch/world.launch): Launch my_gokart mode in Gazebo world with building and plugins.  
- [my_gokart.gazebo](/catkin_ws/src/my_gokart/urdf/my_gokart.gazebo): Define my_gokart URDF model plugins.  
- [my_gokart.xacro](/catkin_ws/src/my_gokart/urdf/my_gokart.xacro): Define my_gokart URDF model.  
- [empty.world](/catkin_ws/src/my_gokart/worlds/empty.world): Gazebo world file that includes nothing.  
- [myoffice.world](/catkin_ws/src/my_gokart/worlds/myoffice.world): Gazebo world file that includes the models.  
- [CMakeLists.txt](/catkin_ws/src/my_gokart/CMakeLists.txt): File to link the C++ code to libraries.  
- [robot_description.launch](/catkin_ws/src/my_robot/launch/robot_description.launch): Create robot model in Gazebo world.  
- [hokuyo.dae](/catkin_ws/src/my_robot/meshes/hokuyo.dae): Hokuyo LiDAR sensor mesh model.  
- [my_robot.gazebo](/catkin_ws/src/my_robot/urdf/my_robot.gazebo): Define my_robot URDF model plugins.  
- [my_robot.xacro](/catkin_ws/src/my_robot/urdf/my_robot.xacro): Define my_robot URDF model.  
- [amcl.launch](/catkin_ws/src/my_robot/launch/amcl.launch): Launch AMCL node
- [myoffice.pgm](/catkin_ws/src/my_robot/maps/myoffice.pgm): Generated myoffice map
- [myoffice.yaml](/catkin_ws/src/my_robot/maps/myoffice.yaml): Info for myoffice map
- [default.rviz](/catkin_ws/src/my_robot/rviz/default.rviz): Default rviz
- [map.pgm](/catkin_ws/src/pgm_map_creator/maps/map.pgm): Generated myoffice map
- [localization.launch](/catkin_ws/src/my_robot/launch/localization.launch): Launch localization node
- [mapping.launch](/catkin_ws/src/my_robot/launch/mapping.launch): Launch mapping node

## Run the project  
* Clone this repository
```
https://github.com/jinchaolu/RoboND-Term1-P4-Map-My-World.git
```
* Open the repository and make  
```
cd /home/workspace/RoboND-Term1-P4-Map-My-World/catkin_ws/
catkin_make
```
* Launch my_robot in Gazebo to load both the world and plugins  
```
roslaunch my_robot world.launch
```  
* Launch teleop_twist_keyboard node, open a new terminal, enter  
```
cd /home/workspace/RoboND-Term1-P4-Map-My-World/catkin_ws/
source devel/setup.bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```  
* Launch teleop_twist_keyboard node, open a new terminal, enter  
```
cd /home/workspace/RoboND-Term1-P4-Map-My-World/catkin_ws/
source devel/setup.bash
roslaunch my_robot mapping.launch
```  
* Testing  
Send move command via teleop package to control your robot and observe real-time visualization in the environment `rtabmapviz`.  
rtabmap-databaseViewer ~/.ros/rtabmap.db

* View database
Once you statisfied with your move, press `Ctrl + c` to exit then view your database with
```
rtabmap-databaseViewer ~/.ros/rtabmap.db
```
Remember to rename your `~/.ros/rtabmap.db` before your next attempt since it will be deleted due to the launch file setting in `mapping.launch`

## Tips  
1. It's recommended to update and upgrade your environment before running the code.  
```bash
sudo apt-get update && sudo apt-get upgrade -y
```
2. Remember to rename your `~/.ros/rtabmap.db` before your next attempt since it will be deleted due to the launch file setting in `mapping.launch`

## Code Style  
Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Rubric  
### 1. Basic Requirements  
#### 1.1 Did the student submit all required files?  
Yes, he did.   
### 2. Simulation Setup
#### 2.1 Did the student set up the simulation environment properly?  
Yes, he did.  
#### 2.2 Is the student's simulation suitable for mapping task?  
Yes, it is.  
### 3. Mapping Package  
#### 3.1 Does the student correctly build all required launch files for RTAB-Mapping?  
Yes, he does.  
### 4. Mapping Accuracy  
#### 4.1 Was the student able to generate a 3D map using RTAB-Map?  
Yes, he was.  
#### 4.2 Does the student's 3D map portray environment characteristics?  
Yes, he does.  