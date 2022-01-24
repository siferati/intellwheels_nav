# Intellwheels Navigation

## Setup

This repository is a fork from [Intellwheels Navigation](https://github.com/siferati/intellwheels_nav) check the repository for

## Python and requirements 

```
ROS Noetic

Python 2.7

virtualenv

Keras = 2.1.5

tensorflow-gpu = 1.14.0
```

### Open AI ROS

```
git clone https://bitbucket.org/theconstructcore/openai_ros/src/kinetic-devel/

catkin_make

source devel/setup.bash

rosdep install openai_ros
```

More information at [(http://wiki.ros.org/openai_ros](http://wiki.ros.org/openai_ros)


### Virtual environment

To install the correct packages create a virtual environment and run the pip install

```
virtualenv env --python=python2.7

source env/bin/activate

pip install -r requirements.txt

```

**NOTE: every time that you need to launch with DQN you have to activate the virtual environment before**


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

```

Terminal 1: 

$ roslaunch intellwheels_multi_chairs empty_office.launch

Terminal 2: 

$ source env/bin/activate

$ roslaunch intellwheels_rl robot1_qlearn.launch


Terminal 2: 

$ source env/bin/activate

$ roslaunch intellwheels_r2 robot1_qlearn.launch

```

#### DQN

To run the DQN it is necessary to activate the virtual enviromnt

```

Terminal 1: 

$ roslaunch intellwheels_multi_chairs empty_office.launch

Terminal 2: 

$ source env/bin/activate

$ roslaunch intellwheels_rl robot1_dqn.launch


Terminal 2: 

$ source env/bin/activate

$ roslaunch intellwheels_r2 robot1_qdn.launch

```

#### DQN - Test/Deploy

To run the DQN it is necessary to activate the virtual enviromnt

```

Terminal 1: 

$ roslaunch intellwheels_multi_chairs empty_office.launch

Terminal 2: 

$ source env/bin/activate

$ roslaunch intellwheels_rl robot1_dqn_test.launch


Terminal 2: 

$ source env/bin/activate

$ roslaunch intellwheels_r2 robot2_dqn_test.launch

```


## Other experiments

### Run one robot

To run the DQN it is necessary to activate the virtual enviromnt


```

Terminal 1: 

$ roslaunch intellwheels_multi_chairs empty_office_robot1.launch

Terminal 2: 

$ source env/bin/activate

$ roslaunch intellwheels_rl robot1_dqn_test.launch

```


### Run without gazebo gui

```
Terminal 1:  roslaunch intellwheels_multi_chairs empty_office_no_gazebo.launch

```

## How to use multple robots from the same model in Gazebo

(...)


## Credits

**DQN**

Book: "Hands-On ROS for Robotics Programming"

ISBN 9781838551308

[https://github.com/ROBOTIS-GIT/turtlebot3_machine_learning](https://github.com/ROBOTIS-GIT/turtlebot3_machine_learning) 

**Q-Learning algorithm**

[https://github.com/vmayoral/basic_reinforcement_learning ](https://github.com/vmayoral/basic_reinforcement_learning )


