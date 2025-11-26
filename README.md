# aerial_robotics

## :package: About

This package is supposed to be used together with PX4-Autopilot package. 
To perform the correct use of the files contained in this repository, ensure to place them in the correct following path:
PX4-Autopilot/Tools/simulation/gz/models/**dmmf_drone**
PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/**CMakeLists.txt**
`PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/`**4022_gz_dmmf_drone**
`PX4-Autopilot/src/modules/uxrce_dds_client/`**dds_topic.yaml**

reganding the bag files and the `aerial_robotics` folder, place them in the folder connected with your aerial_robotic's docker image.

## :pizza: Simulate the pizza drone

To simulate our drone, follow these steps:
1) start QGruond;
2) open a terminal and move to the folder /home/user/ros2_ws, then build and source the workspace;
3) move to the folder /src/PX4-Autopilot and execute:
```
make px4_sitl gz_dmmf_drone
```
   this will start a Gazebo simulation in which you can visualize the movement of the drone.

Now you can enjoy your pizza flight using the sticks in QGround.

If Gazebo fails to start, you can resolve the issue by changing the default world used by the simulation. To do this:

1. Navigate to:
   `PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes`

2. Open the file `4022_gz_dmmf_drone`.

3. In the first few lines, locate the occurrence of `baylands` and replace it with `default`.

4. Try launching Gazebo again.

## :arrow_down: Test the force land
First, perform the preparation steps already seen, opening QGround and three different terminals and ensuring that you have set the source in each one.

1) in order to bridge the uORB topics in ros2 topics, in the first terminal move to the folder /src/aerial_robotics and execute:
```
./DDS_run.sh
```
   ensure you've made the file executable with
```
chmod +x DDS_run.sh
```
2) in the second one, launch the simulation of the drone with the make command;
3) in the third one, from the ros workspace, run the `force_land_node` with the command:
```
ros2 run force_land_node force_land_node
```
Then try to force the land moving your drone futher then 20 meters of altitude. Interrupt the procedure and verify that, after regaining altitude, it does not start again, unless you decide to land.

## 🌀 Offboard mode
Again, perform the preparation steps already seen, opening QGround and three different terminals and ensuring that you have set the source in each one.

1) in the first terminal run the DDS as seen before;
2) in the second one, launch the simulation of the drone with the make command and then takeoff;
3) wait a few seconds  for the drone to stabilize;
4) in the third one, from the ros workspace, run the node to activate the offboard mode and the execution of the helix trajectory with the command:
```
ros2 run offboard_rl go_to_point
```


