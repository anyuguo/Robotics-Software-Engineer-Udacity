#!/bin/sh

# Create a test_navigation.sh shell script that launches these files
# turtlebot_world.launch: to deploy a turtlebot in your environment
# amcl_demo.launch: to localize the turtlebot
# view_navigation.launch: observe the map in rviz
# add_markers node: add virtual objects/markers in rviz


# Define workspace variable
path_catkin_ws="/home/workspace/catkin_ws"

# Open the workspace, source and launch turtlebot_world.launch
xterm -e "cd ${path_catkin_ws} && source devel/setup.bash && roslaunch turtlebot_gazebo turtlebot_world.launch" &

sleep 5

# Open the workspace, source and launch amcl_demo.launch
xterm -e "cd ${path_catkin_ws} && source devel/setup.bash && roslaunch turtlebot_gazebo amcl_demo.launch map_file:=${path_catkin_ws}/src/map/map.yaml" &

sleep 5

# Open the workspace, source and launch view_navigation.launch
xterm -e "cd ${path_catkin_ws} && source devel/setup.bash && roslaunch turtlebot_rviz_launchers view_navigation.launch" &

sleep 5

# Open the workspace, source and launch add_markers add_markers
xterm -e "cd ${path_catkin_ws} && source devel/setup.bash && rosrun add_markers add_markers" 