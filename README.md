# Intellwheels Navigation

## Setup

Install the [Code Remote - Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) VS Code extension and *clone this repository in container volume.*

**This is only supported for Nvidia GPUs.** If you are using an AMD or Intel GPU you need to edit `.devcontainer/devcontainer.json` to fit your needs - [this](http://wiki.ros.org/docker/Tutorials/Hardware%20Acceleration) is a good place to start.

If you are just looking for a Dockerfile, you can find one inside the `.devcontainer` folder.

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

### intellwheels_gazebo_nav

Navigates the wheelchair in the lab gazebo world.

* `main.launch` - launches everything.

### intellwheels_nav

Navigates the wheelchair in the lab stage world.

* `main.launch` - launches everything.
