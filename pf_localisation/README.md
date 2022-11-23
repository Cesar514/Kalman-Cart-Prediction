# PF Localisation Package (Skeleton Code)

## Particle Filter Exercise

This package implements the particle filter localisation using sensor and motion update from the Pioneer P3-DX robot. The methods in `src/pf_localisation/pf.py` have to be completed correctly to run the node. Read the assignment lab notes for more instructions on how to complete these methods. You can also find documentation regarding each method in the source files.

#### Note:

* You need to make changes **ONLY** in `pf.py` file for completing the localisation package. If you want to change any of the inherited parameters (parameters inherited from the parent class `PFLocaliserBase`), it is best to do so from the child class itself (i.e. in `PFLocaliser` class in `pf.py`).

* However, you may play with different values for parameters in the other files (eg. `sensor_model.py`) for conducting experiments.


### Building Package:

* Move package to your catkin workspace (`src` directory)
* Rebuild catkin workspace 
        
        catkin_make    # ----- run from root directory of catkin workspace

* Compile laser_trace.cpp (provides laser ray tracing) as follows **if you are not using arm system(windows, unix...)**:

        cd <catkin_ws>/src/pf_localisation/src/laser_trace
        ./compile.sh #You may have to '''chmod +x compile.sh'''

* replace `./compile.sh` with `./compilearm.sh`  **if you are using arm system(m1 chip mac)**:

If correctly compiled, you should find `laser_trace.so` in the directory `<catkin_ws>/src/pf_localisation/src/pf_localisation`.
If the ***code does not compile*** you need to install PythonBoost from https://github.com/boostorg/python. This requires the download and compiling of Boost and installation of Faber.

### Running the node:

#### On real robot:

        roscore  # ----- not necessary if roslaunch is called before running any nodes with rosrun
        roslaunch socspioneer p2os_laser.launch
        roslaunch socspioneer teleop_joy.launch # ----- for teleoperation control (if implementing automatic collision avoidance node, run that instead)
        rosrun map_server map_server <path_to_your_map_yaml_file>
        rosrun pf_localisation node.py    # ----- requires laser_trace, and completed pf.py methods.

#### In simulated world:

The localisation node can be tested in stage simulation (without the need for robot).

        roscore
        rosrun map_server map_server <catkin_ws>/map.yaml
        rosrun stage_ros stageros <catkin_ws>/src/socspioneer/data/meeting.world
        roslaunch socspioneer keyboard_teleop.launch  # ---- run only if you want to move robot using keyboard 
        rosrun pf_localisation node.py    # ----- requires laser_trace, and completed pf.py methods.

**Don't forget to make node.py executable by using ```chmod +x node.py```**

### Published Topics:

Running the node successfully will publish the following topics:

* `/map` 
* `/amcl_pose` 
* `/particle_cloud`

All of these can be visualised in RViz by adding the appropriate Views.


### Advice:

* Once you have a basic understanding of what you have to do in the `PFLocaliser` class of `pf.py` file, it is a good idea to look at its parent class `PFLocaliserBase` in `pf_base.py` to see how the other parts of the localisation algorithm is implemented. 
* You may also go through `sensor_model.py` to see how the sensor model is coded, and how the model update is implemented. The different parameter values that were empirically chosen can also be found here. 
* Read through `node.py` to understand how the node is impelemented to perform the update whenever new information is available. You can also see how `rospy logging` is used for logging useful information to console. You may also use this for debugging purposes.
