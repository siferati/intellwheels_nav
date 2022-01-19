# Intellwheels Navigation

## Setup



This repository is a fork from [Intellwheels Navigation](https://github.com/siferati/intellwheels_nav) check the repository for

## Python and requirements 

ROS Noetic

Python 2.7

virtualenv


To install the correct packages create a virtual environment and run the pip install

    virtualenv env --python=python2.7

    source env/bin/activate

    pip install -r requirements.txt


## Dependencies

```
$ rosdep install --from-paths src --ignore-src -r -y
```

## Usage

```
$ catkin build
$ source devel/setup.bash
$ roslaunch <package> <file>
```

## Packages & Launch Files

### intellwheels_desc

Describes the wheelchair 3d robot model.

* `wheelchair.launch` - loads the robot description and publishes the joints and robot states.
* `debug.launch` - useful for debugging the robot model. Spawns the wheelchair in an empty gazebo world with manual steering.

### intellwheels_multi_chair

Open a gazebo and a rviz with two chairs publish the joints to the ROS



### intellwheels_move_to_goal

Simple example using the stack navigation that move the two chairs independently to a predefined goal


### intellwheels_rl

Launch agent with reinforcement learning capabilities using DQN and Q-Learning

#### Q-Learning

...

#### DQN

...


## Other experiments

- run one robot
- run without gazebo gui

## Polish and errors
...


## Credits

...