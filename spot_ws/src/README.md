# Simulation tools for Boston Dynamics' Spot

The Spot package, model and basic control inputs, is based on [SpotMicro project](https://github.com/OpenQuadruped/spot_mini_mini) and [SpotControl](https://github.com/SoftServeSAG/spot_simulation/tree/spot_control)


## Installation
This simulation uses ROS-Noetic and Gazebo. You will need to have a catkin_ws 

Run catkin_make and source devel/setup.bash before starting simulation 

## Start world in Gazebo
```bash
roslaunch spot_ws humanworld.launch
```
This launches a world with several humans walking or running around an empty world. The file humans.world can be edited to add/ remove move humans. These models are spawned animated humans based on [Gazebo Animated Actor Tutorial](https://classic.gazebosim.org/tutorials?tut=actor&cat=build_robot)

## Spawn Spot robot
```bash
roslaunch spot_ws robot.launch
```
This spawns the spot robot into the world. 

# Simple Moves
```bash
roslaunch spot_ws simple_moves.launch
```
This is a simple node that makes robot perform several actions such as standing up, sitting down or giving the left paw. The resources above will be used to write more complex algorithms to move the robot. 






