# Using real_data files

This folder contains all the material you would need to test your pf_localisation code against data collected from a real Pioneer 3DX robot moving around the lower ground floor of the CS building. The data is unfiltered and will therefore present several challenges that come up when working with real (unsimulated) robots in the real world, such as sensor noise.

Make sure you have `map_server` and `amcl` installed (`sudo apt install ros-melodic-map-server ros-melodic-amcl`) before proceeding with the instructions here.

## Using the map server

For localising the robot, the map of the world has to be known. This can be published using the `map_server` in ROS, which can then be used by other ROS nodes (such as your pf_localisation node, which is already set up to use the published map data).

For starting the map server, run `rosrun map_server map_server map.yaml`. You can test if the map has started correctly by opening Rviz (`rosrun rviz rviz` or simply `rviz`), and adding a display for Map (set topic to `/map`).

## Reading the recorded data

The folder contains 4 `trace[n].bag` files (`n=0,1,2,3`). Each file contains the recording of `/base_scan` (laser data) and `/odom` (odometry data) topics from the real robot collected while navigating different paths along the provided map.

To play back any bag file run `rosbag play <file>`. In another terminal try listing the available topics (`rostopic list`) and try listening to each (`rostopic echo <topic_name>`).

*NOTE: The file `trace0.bag` is probably the best to start testing your algorithm with. It has simpler and slower motions, with mostly motion in straight lines and only a few rotational motions.*

## Localisation using in-built AMCL algorithm

ROS has a native implementation of AMCL (Adaptive Monte-Carlo Localisation). This can be used as a baseline to compare the performance of your localisation algorithm.

To run the AMCL ROS node:

- Start the map server with the provided map
- Run AMCL node: `rosrun amcl amcl scan:=base_scan # this is letting amcl that our laser scanner measurements comes from the topic base_scan`
- Start Rviz.
- Set the Fixed Frame to `/map`.
- Add a Map view listening on the `/map` topic.
- Add a Pose Array view listening on the `/particlecloud` topic.
- Finally, click the `2D Pose Estimate` button on the top bar on RViz, then draw an arrow on the map showing the approximate location and direction in which the robot is facing (see Figure below). This is the initial guess for the localisation algorithm (See note below).
- Now run rosbag using any of the bag files provided. This will provide the laser data and odometry measurements required by the localisation algorithm.

You should see the particle cloud move according to the present state of the AMCL algorihthm. Also note that AMCL publishes its best guess in the topic `/amcl_pose`. This can be viewed in RViz by adding a Pose or PoseWithCovariance view that subscribes to this topic. *NOTE: Your pf_localisation node will also publish a similar best guess which can be viewed in RViz.*

## Regarding Initial Guess for the localisation algorithm

Depending on the implementation, the initial guess provided to the localisation algorithm is very important. Having no idea about the initialisation or starting with a very wrong initial guess falls under the "Kidnapped Robot" problem that your algorithm will be tested on during the demonstration. You can try giving the wrong initial guess for AMCL, and see that it fails most of the time in the beginning, but may converge later on.
Recovering from a wrong initialisation and wrong localisation depends on how you implement your particle initialisation and resampling.

In all the provided traces, the robot's true starting pose is approximately the same (shown in Figure). However, to test the robustness of your algorithm to the "Kidnapped Robot" problem, you should also test with wrong initial poses.

![img](rviz_initial_pose.png)

### Note

**It is very likely that the parameters or parts of code that you used for the simualation data do not work with the real data, mainly due to the fact that the sensors are less perfect, and also may have completely different ranges. You may have to find a way to model/deal with noisy data from the sensors (which were not present in the simulation data). What other effects of noise do you observe in the provided dataset?**
