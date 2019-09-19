This is the code accompanyting the paper Crashing to Learn, Learning to Survive: Planning in dynamic environments via domain randomization. We assume that you have already build a catkin workspace in ROS.

## Prerequisite packages

Openai Gym

```
pip install gym

```
Custom gym environemnt in ROS gazebo for mobile robot navigation

```
https://github.com/sarath-menon/gym_mobilerobot.git
```

Turtlebot3 packages

```
https://github.com/sarath-menon/turtlebot3.git
https://github.com/sarath-menon/turtlebot3_simulations.git
```

## Download and build package

cd ~/catkin_ws/src/
git clone https://github.com/sarath-menon/rl_cheap.git
cd ..
catkin_make
```

## Start training

First train in obstacle free environemnt

```
roslaunch turtlebot3_gazebo train_obstacle_free.launch
```
Then train in environemnt with dyanamic obstacles
```
roslaunch turtlebot3_gazebo custom_collision.launch         
```
