# RoboND-Term1-P5-Home-Service-Robot
Project 5 of Udacity Robotics Software Engineer Nanodegree
![Overview](/videos/Term1-Project5-Home-Service-Robot-Demo.gif)  

## Overview  
In this project, you will use everything you learned in the Nanodegree Program to build a Home Service Robot in ROS.  
### Mapping  
You will create a `test_slam.sh` script file and launch it to manually test SLAM.  
A functional map of the environment should be created which would be used for localization and navigation tasks.  
### Localization and Navigation  
You will create a `test_navigation.sh` script file to launch it for manual navigation test.  
Your robot should be able to navigate in the environment after a 2D Nav Goal command is issued.  
You will create a `pick_objects.sh` file that will send multiple goals for the robot to reach.  
The robot travels to the desired pickup zone, displays a message that it reached its destination, waits 5 seconds, travels to the desired drop off zone, and displays a message that it reached the drop off zone."  
### Home Service Functions  
You will create a `add_marker.sh` file that will publish a marker to rviz.  
The marker should initially be published at the pickup zone. After 5 seconds it should be hidden. Then after another 5 seconds it should appear at the drop off zone.
The student should write a home_service.sh file that will run all the nodes in this project.  
The student's home service robot should be simulated as follow:  
* Initially show the marker at the pickup zone.
* Hide the marker once your robot reach the pickup zone.
* Wait 5 seconds to simulate a pickup.
* Show the marker at the drop off zone once your robot reaches it.


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
3. On the command line and execute  
```bash
sudo apt-get update && sudo apt-get upgrade -y
```
4. On the command line and execute  
```
cd RoboND-Term1-P5-Home-Service-Robot/catkin_ws/src  
git clone https://github.com/ros-perception/slam_gmapping.git  
git clone https://github.com/turtlebot/turtlebot.git  
git clone https://github.com/turtlebot/turtlebot_interactions.git  
git clone https://github.com/turtlebot/turtlebot_simulator.git  
```
5. Build and run your code.  

## Project Description  
Directory Structure  
```
.Home-Sevice-Robot                                        # Home Service Robot Project
├── catkin_ws                                             # Catkin workspace
│   ├── src
│   │   ├── add_markers                                   # add_markers package        
│   │   │   ├── launch
│   │   │   │   ├── view_home_service_navigation.launch   # launch file for home service robot demo
│   │   │   ├── src
│   │   │   │   ├── add_markers.cpp                       # source code for add_markers node
│   │   │   │   ├── add_markers_demo.cpp                  # source code for add_markers_demo
│   │   ├── pick_objects                                  # pick_objects package     
│   │   │   ├── src
│   │   │   │   ├── pick_objects.cpp                      # source code for pick_objects node
│   │   │   │   ├── pick_objects_demo.cpp                 # source code for pick_objects_demo
│   │   ├── rvizConfig                                    # rvizConfig package        
│   │   │   ├── home_service_rvizConfig.rviz              # rvizConfig file for home service robot demo  
│   │   ├── scripts                                       # shell scripts files
│   │   │   ├── add_marker.sh                             # shell script to model virtual objects  
│   │   │   ├── home_service.sh                           # shell script to launch home service robot demo  
│   │   │   ├── pick_objects.sh                           # shell script to send multiple goals  
│   │   │   ├── test_navigation.sh                        # shell script to test localization and navigation
│   │   │   ├── test_slam.sh                              # shell script to test SLAM
│   │   ├── slam_gmapping                                 # gmapping_demo.launch file
│   │   ├── turtlebot                                     # keyboard_teleop.launch file
│   │   ├── turtlebot_interactions                        # view_navigation.launch file
│   │   ├── turtlebot_simulator                           # turtlebot_world.launch file package        
│   │   ├── CMakeLists.txt                                # compiler instructions
├── videos                                                # Videos
│   ├── Term1-Project5-Home-Service-Robot-Demo.gif        # Videos for overview
```
- [view_home_service_navigation.launch](/catkin_ws/src/add_markers/launch/view_home_service_navigation.launch): Launch rviz with specify rviz configuration file  
- [add_markers.cpp](/catkin_ws/src/pick_objects/src/add_markers.cpp): C++ script, communicate with `pick_objects` node and control the marker appearance to simulate object pick up and drop off  
- [add_markers_demo.cpp](/catkin_ws/src/pick_objects/src/add_markers_demo.cpp): C++ script, control the marker appearance to simulate object pick up and drop off  
- [pick_objects.cpp](/catkin_ws/src/pick_objects/src/pick_objects.cpp): C++ script, communicate with `add_markers` node and command the robot to pick up the object  
- [pick_objects_demo.cpp](/catkin_ws/src/pick_objects/src/pick_objects_demo.cpp): C++ script, command the robot to pick up the object  
- [home_service_rvizConfig.rviz](/catkin_ws/src/rvizConfig/home_service_rvizConfig.rviz): rvizConfig file for home service robot demo which contained `markers` option  
- [add_marker.sh](/catkin_ws/src/scripts/add_marker.sh): Shell script file to deploy a turtlebot inside your environment, model a virtual object with markers in `rviz`.  
- [home_service.sh](/catkin_ws/src/scripts/home_service.sh): Shell script file to deploy a turtlebot inside your environment, simulate a full home service robot capable of navigating to pick up and deliver virtual objects.  
- [pick_objects.sh](/catkin_ws/src/scripts/pick_objects.sh): Shell script file to deploy a turtlebot inside your environment, communicate with the ROS navigation stack and autonomously send successive goals for your robot to reach.  
- [test_navigation.sh](/catkin_ws/src/scripts/test_navigation.sh): Shell script file to deploy a turtlebot inside your environment, pick two different goals and test your robot's ability to reach them and orient itself with respect to them.  
- [test_slam.sh](/catkin_ws/src/scripts/test_slam.sh): Shell script file to deploy a turtlebot inside your environment, control it with keyboard commands, interface it with a SLAM package, and visualize the map in `rviz`  

- [CMakeLists.txt](/catkin_ws/src/CMakeLists.txt): File to link the C++ code to libraries.  
- [Term1-Project5-Home-Service-Robot-Demo.gif](/videos/Term1-Project5-Home-Service-Robot-Demo.gif): An demo video for final home service robot run  

## Run the project  
* Clone this repository
```
git clone https://github.com/jinchaolu/RoboND-Term1-P5-Home-Service-Robot.git
```
* Navigate to the `src` folder and clone the necessary repositories  
```
cd RoboND-Term1-P5-Home-Service-Robot/catkin_ws/src  
git clone https://github.com/ros-perception/slam_gmapping.git  
git clone https://github.com/turtlebot/turtlebot.git  
git clone https://github.com/turtlebot/turtlebot_interactions.git  
git clone https://github.com/turtlebot/turtlebot_simulator.git  
```
* Open the repository, make and source  
```
cd /home/workspace/RoboND-Term1-P5-Home-Service-Robot/catkin_ws/
catkin_make
source devel/setup.bash
```
* Launch the home service robot
```
./src/scripts/home_service.sh
```
* Done. 

## Tips  
1. It's recommended to update and upgrade your environment before running the code.  
```
sudo apt-get update && sudo apt-get upgrade -y
```
2. If your system python version from miniconda is python3 while the ros packages and tf are python2. A hack is to just set the system python to python2 via symbol link. Run the following commands to resolve it  
```
ln -s /usr/bin/python2 /root/miniconda3/bin/python
```
3. How to setup your environment at start up.  
```
Add the following line into the /home/workspace/.student_bashrc
export PYTHONPATH=$PYTHONPATH:/usr/lib/python2.7/dist-packages  
pip install catkin_pkg  
pip install rospkg  
```
4. How create package with dependencies  
```
catkin_create_pkg pick_objects move_base_msgs actionlib roscpp  
catkin_create_pkg add_markers roscpp visualization_msgs  
```
5. How to visualize your marker in the rviz  
To see the marker(virtual objects) demo, in addition to running the `./add_marker.sh`, you will need to manually add a 'Marker' in rviz with the following steps:  
* Find your rviz window  
* In the left bottom panel, click "Add" button  
* In 'By display type' tab, navigate the tree to 'rviz' then 'Marker'  
* Click 'OK' button  
* Done, you should see the marker(virtual objects) appear, disappear then appear again  

## Code Style  
Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Rubric  
### 1. Basic Requirements  
#### 1.1 Did the student submit all required files?  
Yes, he did.   
### 2. Simulation Setup
#### 2.1 Did the student set up the simulation environment properly?  
Yes, he did.  
### 3. Mapping  
#### 3.1 Did the student's mapping function work properly?  
Yes, he did.  
#### 3.2 Did the student create a map using SLAM?  
Yes, he did.  
### 4. Localization and Navigation  
#### 4.1 Was the student's navigation stack configured properly?  
Yes, he was.  
#### 4.2 Did the student's goal node function properly?  
Yes, he did.  
### 5. Home Service Functions  
### 5.1 Did the student create virtual object with markers?  
Yes, he did.  
### 5.2 Does the student's robot perform home service tasks correctly?  
Yes, he was.  
### 5.3 Did the student include a write-up explaining the packages used to achieve home service functionalities?  
Yes, he did.  