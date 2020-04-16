# indoor_service_robot
This repo contains a simulation environment of a turtlebot and a U-shape office. A map is created under src/map/ directory by the help of discovering the environment with gmapping slam algorithm. After a map is extracted, turtlebot localize itself with a localization algorithm(amcl). add_marker node adds a marker on Rviz environment given coordinates at add_markers package. Then pick_objects package dives in and turtlebot moves through to marker. After robot reaches first coordinates, marker is removed and wait 5 seconds. Finally robot moves to drop off zone and while odom topic meets the goal coordinates, marker is shown up on Rviz.
### Directory Structure
```
    Indoor Service Robot
    ├──                                # Official ROS packages
    |
    ├── slam_gmapping                  # gmapping_demo.launch file                   
    │   ├── gmapping
    │   ├── ...
    ├── turtlebot                      # keyboard_teleop.launch file
    │   ├── turtlebot_teleop
    │   ├── ...
    ├── turtlebot_interactions         # view_navigation.launch file      
    │   ├── turtlebot_rviz_launchers
    │   ├── ...
    ├── turtlebot_simulator            # turtlebot_world.launch file 
    │   ├── turtlebot_gazebo
    │   ├── ...
    ├──                                # Your packages and direcotries
    |
    ├── map                          # map files
    │   ├── ...
    ├── scripts                   # shell scripts files
    │   ├── ...
    ├──rvizConfig                      # rviz configuration files
    │   ├── ...
    ├──pick_objects                    # pick_objects C++ node
    │   ├── src/pick_objects.cpp
    │   ├── ...
    ├──add_markers                     # add_marker C++ node
    │   ├── src/add_markers.cpp
    │   ├── ...
    └──

```

### Launching the scripts

#### Pre-ops 
```sh
$ cd /home/user/ros_catkin_ws_dir/src/
$ git clone https://github.com/canersu/ball_chaser.git
$ cd /home/user/ros_catkin_ws_dir/
$ catkin_make
```

#### launch.sh
The script below brings Gazebo, roscore and Rviz environments.
```sh
$ cd /home/user/ros_catkin_ws_dir/src/ && ./launch.sh
```

#### test_slam.sh
The script below launches Gazebo, Turtlebot, keyboard_teleop and Rviz packages. It leads to create the map of building by navigating Turtlebot using keyboard commands.
```sh
$ cd /home/user/ros_catkin_ws_dir/src/ && ./test_slam.sh
```

#### test_navigation.sh
The script below brings Gazebo, Turtlebot, amcl and Rviz packages. It shows the robot's position and orientation.
```sh
$ cd /home/user/ros_catkin_ws_dir/src/ && ./test_navigation.sh
```

#### pick_objects.sh
The script below brings Gazebo, Turtlebot, amcl, Rviz and pick_objects packages. It leads the robot to specified coordinates in source file.
```sh
$ cd /home/user/ros_catkin_ws_dir/src/ && ./pick_objects.sh
```

#### add_marker.sh
The script below brings Gazebo, Turtlebot, amcl, Rviz and add_markers packages. By launching this script, set of actions will occur in order: A marker is spawned in first coordinate for 5 seconds, disappears for 5 seconds and spawns in goal coordinates.
```sh
$ cd /home/user/ros_catkin_ws_dir/src/ && ./add_marker.sh
```

#### home_service.sh
The script below brings Gazebo, Turtlebot, amcl, Rviz, pick_objects and add_markers packages. By launching this script, set of actions will occur in order: A marker is spawned in first coordinate, robot moves toward the marker, marker disappears and wait for 5 seconds, robot moves through goal position, marker spawns as the robot reaches goal coordinates.
```sh
$ cd /home/user/ros_catkin_ws_dir/src/ && ./home_service.sh
```
