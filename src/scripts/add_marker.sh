#!/bin/sh

export TURTLEBOT_GAZEBO_MAP_FILE=/home/workspace/catkin_ws/src/map/office_map.yaml
xterm  -e "source /home/workspace/catkin_ws/devel/setup.bash; roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 5
xterm  -e "roslaunch turtlebot_gazebo amcl_demo.launch" &
sleep 2
xterm  -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 2
xterm  -e "roslaunch add_markers add_markers.launch" &

