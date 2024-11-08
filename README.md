We have two ROS packages so far:

 - crazyflie_gazebo
 - controllers

crazyflie_gazebo contains the SDF files for the drone, ros-gazebo bridge configuration and a launch file that opens the simulation and loads the drone.
This package also contains a node that allows us to control the drone in simulation using the keyboard.

controllers package contains our controllers. It currently has one node which publishes individual motor velocities to the drone.

By editing the launch file we can choose which node the load.

There are currently two SDF files for the drone in the crazyflie_gazebo, however we can combine them.
- One of the SDF files allows us to send twist commands, other allows us to send motor velocities.

To run the simulation simply do:
```
cd crazyflie_simulation

colcon build 

ros2 launch crazyflie_gazebo crazyflie_gazebo.launch.py 
```
Then you can enable the keyboard by:

ros2 run teleop_twist_keyboard teleop_twist_keyboard

Don't forget to change the paths in the crazyflie_world.sdf file. (It would be nice if we can make those relative paths)
