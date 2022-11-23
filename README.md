
Commands list

% START ROSCORE %
cd catkin_ws
roscore

% LOAD MAP DATA %
crtl + shift + t on terminal (Open a new terminal)
rosrun map_server map_server src/final_project/data/market.yaml

% LOAD MAP SCREEN%
crtl + shift + t on terminal (Open a new terminal)
rosrun stage_ros stageros src/final_project/data/market.world

% START TELEOP SCRIPT %
crtl + shift + t on terminal (Open a new terminal)
cd ..
catkin_make
source devel/setup.bash
roslaunch socspioneer keyboard_teleop.launch

% START HUMAN MOVEMENT SCRIPT%
crtl + shift + t on terminal (Open a new terminal)
rosrun socspioneer twist.py
 --> After this you can go to teleop window to move human

% SEE CONNECTED NODES %
rosrun rqt_graph rqt_graph

% START RVIZ %
rosrun rviz rviz
