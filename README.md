# Path planning

## Commands list

### START ROSCORE

```
cd catkin_ws
roscore
```

### LOAD MAP DATA
> crtl + shift + t on terminal (Open a new terminal)
```
rosrun map_server map_server src/final_project/data/market.yaml
```

### LOAD MAP SCREEN%
> crtl + shift + t on terminal (Open a new terminal)
```
rosrun stage_ros stageros src/final_project/data/market.world
```

### START TELEOP SCRIPT
> crtl + shift + t on terminal (Open a new terminal)
```
cd ..
catkin_make
source devel/setup.bash
roslaunch socspioneer keyboard_teleop.launch
```

### START HUMAN MOVEMENT SCRIPT%
> crtl + shift + t on terminal (Open a new terminal)
```
source devel/setup.bash
rosrun final_project twist.py
```
> After this you can go to teleop window to move human

### SEE CONNECTED NODES
```
rosrun rqt_graph rqt_graph
```

### START RVIZ
```
rosrun rviz rviz
```

### RUN ROBOT CONTROL
```
source devel/setup.bash
rosrun final_project robotMove.py
```

### RUN ASTAR
```
source devel/setup.bash
rosrun final_project pathPlanning.py
```

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

### Test Worlds (LARGE)
```
rosrun stage_ros stageros src/final_project/data/Worlds/LARGEMAP/market#.world
```
> Remember to check speeds comparison for Manhattan and Euclidean Distance

### For Visual Testing
```
source devel/setup.bash
rosrun map_server map_server src/final_project/data/marketMed.yaml

source devel/setup.bash
rosrun stage_ros stageros src/final_project/data/marketMed.world
```
