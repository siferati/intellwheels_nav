# Intellwheels Navigation

## Setup

Install the [Code Remote - Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) extension and *clone this repository in container volume.*

**This is only supported for Nvidia GPUs.** If you are using an AMD or Intel GPU you need to edit `.devcontainer/devcontainer.json` to fit your needs - [this](http://wiki.ros.org/docker/Tutorials/Hardware%20Acceleration) is a good place to start.

If you are just looking for a Dockerfile, you can find one inside the `.devcontainer` folder.

## Usage

```
> catkin_make
> source devel/setup.bash
> roslaunch intellwheels_nav main.launch
```

## Dependencies (non-Docker)

`sudo apt install ros-melodic-stage ros-melodic-stage-ros ros-melodic-global-planner ros-melodic-teb-local-planner ros-melodic-move-base ros-melodic-map-server ros-melodic-amcl ros-melodic-rviz`
