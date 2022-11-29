# Path planning

## Commands list
Remember to always run chmod+x file.py to make the codes work
## Environment ONE Path planning with Position of human with Odometry.
This environment is supposed to allow path planning and following the human around. After this if the user starts the operation of "To paying area" the robot should be able to go to the paying area. When the battery is low, the robot

twist.py
pathPLanning.py
robotMove.py

> START ROSCORE
```
cd catkin_ws
roscore
```
> LOAD MAP DATA
Then open a new terminal
```
rosrun map_server map_server src/final_project/data/market.yaml
```
> LOAD MAP SCREEN
Then open a new terminal
```
rosrun stage_ros stageros src/final_project/data/market.world
```
> START TELEOP SCRIPT
Then open a new terminal
```
source devel/setup.bash
roslaunch socspioneer keyboard_teleop.launch
```
> START HUMAN MOVEMENT SCRIPT
Then open a new terminal
```
source devel/setup.bash
rosrun final_project twist.py
```
After this you can go to teleop window to move human

> SEE CONNECTED NODES
```
rosrun rqt_graph rqt_graph
```

> START RVIZ
```
rosrun rviz rviz
```
Be sure to load the corresponding .rviz file to get needed information displayed
> RUN ROBOT CONTROL
This allows the robot to be controlled by the algorithms
```
source devel/setup.bash
rosrun final_project robotMove.py
```
> RUN ASTAR
Initial path planning Algorithm
```
source devel/setup.bash
rosrun final_project pathPlanning.py
```
## Environment TWO Visual path planning (MAP 500x500)
This environment is supposed to allow path planning giving position of human based on camera robot should be following the human around. After this if the user starts the operation of "To paying area" the robot should be able to go to the paying area. When the battery is low, the robot goes to the closest battery station.


## Environment THREE Visual path planning with prediction (MAP 500x500)
This environment is supposed to allow path planning giving position of human based on camera robot should be following the human around. After this if the user starts the operation of "To paying area" the robot should be able to go to the paying area. When the battery is low, the robot goes to the closest battery station.


## Environment FOUR Visual path planning with Obstacles Update (MAP 500x500)
This environment is supposed to allow path planning giving position of human based on camera robot should be following the human around. After this if the user starts the operation of "To paying area" the robot should be able to go to the paying area. When the battery is low, the robot goes to the closest battery station.

## Environment FIVE Visual path planning with Obstacles Update and prediction(MAP 500x500)
This environment is supposed to allow path planning giving position of human based on camera robot should be following the human around. After this if the user starts the operation of "To paying area" the robot should be able to go to the paying area. When the battery is low, the robot goes to the closest battery station.


### ASTAR FOR MED MAP, PATH PLANNING
```
source devel/setup.bash
rosrun final_project pathPlanning.py
```

### RUN Computer Vision %
```
source devel/setup.bash
rosrun final_project objectMovements.py
```

### RUN Computer Vision Map update
```
source devel/setup.bash
rosrun final_project visualPlanning.py
```

### RUN Computer Vision with MAP REBUILD
```
source devel/setup.bash
rosrun final_project objectMovementsRebuild.py
```

### EXTRA Test Worlds (LARGE) for testing positions
```
rosrun stage_ros stageros src/final_project/data/Worlds/LARGEMAP/market#.world
```
> Remember to check speeds comparison for Manhattan and Euclidean Distance and optimal heuristics


### For Visual Testing
```
source devel/setup.bash
rosrun map_server map_server src/final_project/data/marketMed.yaml

source devel/setup.bash
rosrun stage_ros stageros src/final_project/data/marketMed.world
```
