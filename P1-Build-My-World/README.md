# RoboND-Term1-P1-Build-My-World
Project 1 of Udacity Robotics Software Engineer Nanodegree Program

![screenshot](https://github.com/anyuguo/Robotics-Software-Engineer-Udacity/blob/master/P1-Build-My-World/pic/screenshot.PNG)  

## Overview  
In this project you'll create your simulation world in Gazebo for all your upcoming projects in the [Udacity Robotics Software Engineer Nanodegree Program](https://www.udacity.com/course/robotics-software-engineer--nd209).  
1. Build a single floor wall structure using the **Building Editor** tool in Gazebo. Apply at least one feature, one color, and optionally one texture to your structure. Make sure there's enough space between the walls for a robot to navigate.  
2. Model any object of your choice using the **Model Editor** tool in Gazebo. Your model links should be connected with joints.  
3. Import your structure and two instances of your model inside an empty **Gazebo World**.  
4. Import at least one model from the **Gazebo online library** and implement it in your existing Gazebo world.  
5. Write a C++ **World Plugin** to interact with your world. Your code should display “Welcome to ’s World!” message as soon as you launch the Gazebo world file.  
## Prerequisites/Dependencies  
* Gazebo >= 7.0  
* ROS Kinetic  
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
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
.Build-My-World                    # Build My World Project 
├── model                          # Model files 
│   ├── floor
│   │   ├── model.config
│   │   ├── model.sdf
│   ├── rb
│   │   ├── model.config
│   │   ├── model.sdf
├── script                         # Gazebo World plugin C++ script      
│   ├── welcome.cpp
├── world                          # Gazebo main World containing models 
│   ├── office.world
├── CMakeLists.txt                 # Link libraries 
└── pic 
│   │   ├── screenshot.png
│   │   ├── floorplan.png
```
- [office.world](/world/office.world): Gazebo world file.  
- [floor](/model/floor): Floor structure built by Building Editor of Gazebo.  
- [robot](/model/rb): A robot built by Model Editor of Gazebo.  
- [welcome.cpp](/script/welcome.cpp): Gazebo world plugin C++ script.  
- [CMakeLists.txt](CMakeLists.txt): File to link the C++ code to libraries.  
## Run the project  
* Clone this repository
* At the top level of the project repository, create a build directory:  
```bash
mkdir build && cd build
```
* In `/build` directory, compile your code with  
```bash
cmake .. && make
```
* Export your plugin folder in the terminal so your world file can find it:  
```bash
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:/home/workspace/P1-Build-My-World/build
```
* Launch the world file in Gazebo to load both the world and plugin  
```bash
cd /home/workspace/P1-Build-My-World/world/
gazebo office.world
```

## Tips  
1. It's recommended to update and upgrade your environment before running the code.  
```bash
sudo apt-get update && sudo apt-get upgrade -y
```

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Rubric  
### 1. Basic Requirements  
#### 1.1 Does the project include a world directory containing the Gazebo world file, a model directory containing a structure and an object model files, a script directory containing the C++ plugin code, and a CMakeLists.txt file?  
Yes, it does.  
### 2. Building  
#### 2.1 Does the project include a house with walls?  
Yes, it does.  
### 3. Modeling  
#### 3.1 Does the project include an object built using the Model Editor?  
Yes, it does.  
### 4. Gazebo World  
#### 4.1 Does the project contain a Gazebo world with multiple models?  
Yes, it does.  
### 5. World Plugin  
#### 5.1 Does the project contain a C++ world plugin?  
Yes, it does.  
