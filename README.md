# Intellwheels Navigation


<!-- TOC -->autoauto- [Intellwheels Navigation](#intellwheels-navigation)auto    - [Setup](#setup)auto    - [Python and requirements](#python-and-requirements)auto        - [Open AI ROS](#open-ai-ros)auto        - [Virtual environment](#virtual-environment)auto    - [Dependencies](#dependencies)auto    - [Usage](#usage)auto    - [Packages & Launch Files](#packages--launch-files)auto        - [intellwheels_desc](#intellwheels_desc)auto        - [intellwheels_multi_chair](#intellwheels_multi_chair)auto        - [intellwheels_move_to_goal](#intellwheels_move_to_goal)auto        - [intellwheels_rl](#intellwheels_rl)auto            - [Q-Learning](#q-learning)auto            - [DQN](#dqn)auto            - [DQN - Test/Deploy](#dqn---testdeploy)auto    - [Other experiments](#other-experiments)auto        - [Run gazeo with one robot only](#run-gazeo-with-one-robot-only)auto        - [Run without gazebo gui](#run-without-gazebo-gui)auto    - [How to use multiple robots from the same model in Gazebo](#how-to-use-multiple-robots-from-the-same-model-in-gazebo)auto    - [Code structure](#code-structure)auto    - [Credits](#credits)autoauto<!-- /TOC -->


## Setup

This repository is a fork from [Intellwheels Navigation](https://github.com/siferati/intellwheels_nav) 

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
git clone git clone https://bitbucket.org/theconstructcore/openai_ros.git

catkin_make

source devel/setup.bash

rosdep install openai_ros
```

More information at [http://wiki.ros.org/openai_ros](http://wiki.ros.org/openai_ros)


### Virtual environment

To install the correct packages create a virtual environment and run the pip install.

```
virtualenv env --python=python2.7

source env/bin/activate

pip install -r requirements.txt

```

**NOTE: every time that you need to launch with DQN you have to activate the virtual environment before.**


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

Open a gazebo and a rviz with two chairs publish the joints to the ROS.


### intellwheels_move_to_goal

Simple example using the stack navigation that move the two chairs independently to a predefined goal.

```
**Terminal 1:** 

$ roslaunch intellwheels_multi_chairs empty_office.launch

**Terminal 2:**

$ roslaunch intellwheels_move_goal main.launch
```

### intellwheels_rl

Launch agent with reinforcement learning capabilities using DQN and Q-Learning.

#### Q-Learning

```

**Terminal 1:** 

$ roslaunch intellwheels_multi_chairs empty_office.launch

**Terminal 2:**

$ source env/bin/activate

$ roslaunch intellwheels_rl robot1_qlearn.launch


**Terminal 3:** 

$ source env/bin/activate

$ roslaunch intellwheels_rl robot2_qlearn.launch

```

#### DQN

To run the DQN it is necessary to activate the virtual environment.

```

**Terminal 1:** 

$ roslaunch intellwheels_multi_chairs empty_office.launch


**Terminal 2:**

$ source env/bin/activate

$ roslaunch intellwheels_rl robot1_dqn.launch


**Terminal 3:** 

$ source env/bin/activate

$ roslaunch intellwheels_rl robot2_qdn.launch

```

#### DQN - Test/Deploy

To run the DQN it is necessary to activate the virtual environment.

```

**Terminal 1:** 

$ roslaunch intellwheels_multi_chairs empty_office.launch

**Terminal 2: **

$ source env/bin/activate

$ roslaunch intellwheels_rl robot1_dqn_test.launch


**Terminal 3:** 

$ source env/bin/activate

$ roslaunch intellwheels_rl robot2_dqn_test.launch

```


## Other experiments

### Run gazeo with one robot only

To run the DQN it is necessary to activate the virtual environment.


```

**Terminal 1:** 

$ roslaunch intellwheels_multi_chairs empty_office_robot1.launch

**Terminal 2:**

$ source env/bin/activate

$ roslaunch intellwheels_rl robot1_dqn_test.launch

```


### Run without gazebo gui

```
**Terminal 1:  **

$ roslaunch intellwheels_multi_chairs empty_office_no_gazebo.launch

```

## How to use multiple robots from the same model in Gazebo

It necessary to define a namespace, prefix, a frame and **don't forget to repeat the parameter description inside the namespace with proper perfix !!! **

**-param /robot1/robot_description"**


robots.launch

```
<!-- ...  -->

 <param name="robot_description" command="xacro '$(find intellwheels_desc)/urdf/wheelchair.urdf.xacro'"/> 
  
  <group ns="robot1">
    <param **name="tf_prefix"** value="robot1_tf" />
    <arg name="init_pose" value="-x 1.5 -y -1.0 -z 0 -Y 0.0" />
    <arg name="robot_name"  value="robot1" />
    
    <param name="/robot1/robot_description" command="xacro '$(find intellwheels_desc)/urdf/wheelchair.urdf.xacro'"/>
    <node name="spawn_model" pkg="gazebo_ros" type="spawn_model" respawn="false" output="screen"
          args="$(arg init_pose) -urdf -model $(arg robot_name) -param /robot1/robot_description" />
    <node name="joint_pub" pkg="joint_state_publisher" type="joint_state_publisher"/> 
    <node pkg="robot_state_publisher" type="robot_state_publisher" name="robot_state_publisher" output="screen"/>	
  </group>


  <group ns="robot2">
    <param name="tf_prefix" value="robot2_tf" />
    <arg name="init_pose" value="-x -2.0 -y 0 -z 0 -Y 0.0" />
    <arg name="robot_name"  value="robot2" />
    <param name="/robot2/robot_description" command="xacro '$(find intellwheels_desc)/urdf/wheelchair.urdf.xacro'"/>
    <node name="spawn_model" pkg="gazebo_ros" type="spawn_model" respawn="false" output="screen"
          args="-urdf $(arg init_pose) -model $(arg robot_name) -param /robot2/robot_description" />
    <node name="joint_pub" pkg="joint_state_publisher" type="joint_state_publisher"/> 
    <node pkg="robot_state_publisher" type="robot_state_publisher" name="robot_state_publisher" output="screen"/>
  </group>

   <!-- redirect the scans laser to a topic and redirect de acml and move launchers with proper definition  -->

   <!-- AMCL  -->

  <group>
		<node name="right_front_rplidar_scan_relay_robot1" pkg="topic_tools" type="relay" args="/robot1/right_front_rplidar_scan robot1_base_scan"/>
		<node name="left_back_rplidar_scan_relay_robot1" pkg="topic_tools" type="relay" args="/robot1/left_back_rplidar_scan robot1_base_scan"/>
	</group>
  <include file="$(find intellwheels_multi_chairs)/launch/amcl_robot1.launch" />
  
  
  <group>
		<node name="right_front_rplidar_scan_relay_robot2" pkg="topic_tools" type="relay" args="/robot2/right_front_rplidar_scan robot2_base_scan"/>
		<node name="left_back_rplidar_scan_relay_robot2" pkg="topic_tools" type="relay" args="/robot2/left_back_rplidar_scan robot2_base_scan"/>
	</group>
 
  <include file="$(find intellwheels_multi_chairs)/launch/amcl_robot2.launch" />
  
    <!-- MOVE_BASE-->

  <include file="$(find intellwheels_multi_chairs)/launch/move_base_1.launch" />
  <include file="$(find intellwheels_multi_chairs)/launch/move_base_2.launch"/>

   <!-- ...  -->

```

amcl_robot1.launch


```

<!-- ...  -->

  <arg name="scan_topic"      default="/robot1_base_scan"/>
  <arg name="odom_frame_id"   default="/robot1_tf/odom"/>
  <arg name="base_frame_id"   default="/robot1_tf/base_footprint"/>
  <arg name="global_frame_id" default="map"/>

<!-- ...  -->


```

amcl_robot2.launch

```
<!-- ...  -->

  <arg name="scan_topic"      default="/robot2_base_scan"/>
  <arg name="odom_frame_id"   default="/robot2_tf/odom"/>
  <arg name="base_frame_id"   default="/robot2_tf/base_footprint"/>
  <arg name="global_frame_id" default="map"/>

<!-- ...  -->

```

move_base_1.launch

```

<!-- ...  -->

  <arg name="odom_frame_id"   default="robot1_tf/odom"/>
  <arg name="base_frame_id"   default="robot1_tf/base_footprint"/>
  <arg name="global_frame_id" default="map"/>
  <arg name="odom_topic" default="/robot1/odom" />
  <arg name="laser_topic" default="/robot1/robot1_base_scan" />

<!-- ...  -->

```

move_base_2.launch


```
<!-- ...  -->


  <arg name="odom_frame_id"   default="robot2_tf/odom"/>
  <arg name="base_frame_id"   default="robot2_tf/base_footprint"/>
  <arg name="global_frame_id" default="map"/>
  <arg name="odom_topic" default="/robot2_tf/odom" />
  <arg name="laser_topic" default="/robot2/robot2_base_scan" />

<!-- ...  -->

```


## Code structure 

The most important package is the intellwheels_rl with the following code structure:

```
Code
|
|_src
    |
    |_ **intellwheels_rl**
         |
         |_ config
         |
         |_ launch
         |
         |_ meshes
         |
         |_ metrics
         |
         |_rviz
         |
         |_ save_mode
         |
         |_ srs
         |    |
         |    |_ algorithms
         |    |
         |    |_ robot1
         |    |
         |    |_ robot2
         |    |  
         |    |_ tools
         |
         |_ worlds

```

**launch**: files to start the dqn and qlearn agent in developemnt and test mode

**meshes**: mesh used to represent the goal of the leader chair

**metrics**: *.csv and python code used to plot the graphics

**save_model**: model used to train the DQN agent

**algorithms**: the DQN agent and Q-Learning agent source code 

**robot1**: the main for the robot1 and their environment

**robot2**: the main for the robot2 and their environment

**tools**: classes common for both agents



## Credits

The source code was based on:

**DQN**

Book: "Hands-On ROS for Robotics Programming"

ISBN 9781838551308

[https://github.com/ROBOTIS-GIT/turtlebot3_machine_learning](https://github.com/ROBOTIS-GIT/turtlebot3_machine_learning) 

**Q-Learning algorithm**

[https://github.com/vmayoral/basic_reinforcement_learning ](https://github.com/vmayoral/basic_reinforcement_learning )


