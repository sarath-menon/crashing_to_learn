# Deep RL for mobile robot navigation [ROS - Gazebo]

This is the code accompanyting the paper...

## Prerequisite packages

Openai gym environmentss

```
cd ~/catkin_ws/src/
git clone {link_git}
cd ~/catkin_ws && catkin_make
```
- https://github.com/ROBOTIS-GIT/turtlebot3_machine_learning

## Libraries

[Pytorch]

## ROS 
You can find the packages the I used here:
- https://github.com/ROBOTIS-GIT/turtlebot3
- https://github.com/ROBOTIS-GIT/turtlebot3_msgs
- https://github.com/ROBOTIS-GIT/turtlebot3_simulations

```
cd ~/catkin_ws/src/
git clone {link_git}
cd ~/catkin_ws && catkin_make
```

To install my package you will do the same from above.

## Set State

In: turtlebot3/turtlebot3_description/urdf/turtlebot3_burger.gazebo.xacro.

```
<xacro:arg name="laser_visual" default="false"/>   # Visualization of LDS. If you want to see LDS, set to `true`
```
And
```
<scan>
  <horizontal>
    <samples>360</samples>            # The number of sample. Modify it to 10
    <resolution>1</resolution>
    <min_angle>0.0</min_angle>
    <max_angle>6.28319</max_angle>
  </horizontal>
</scan>
```

## Run Code
I have four stage as in the examples of Robotis. But I dont know yet my code dont have a geat performance in stage 3.

First to run:
```
roslaunch turtlebot3_gazebo turtlebot3_stage_{number_of_stage}.launch
```
In another terminal run:
```
roslaunch project ddpg_stage_{number_of_stage}.launch
```
