# Intellwheels Navigation

## Setup

Install the [Code Remote - Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) extension and import this repository into a container. Don't forget to create the catkin workspace!

**This is only supported for Nvidia GPUs.** If you are using an AMD or Intel GPU you need to edit `.devcontainer/devcontainer.json` to fit your needs - [this](http://wiki.ros.org/docker/Tutorials/Hardware%20Acceleration) is a good place to start.

If you are just looking for a Dockerfile, you can find it inside the `.devcontainer` folder.

## Usage

`roslaunch intellwheels_nav main.launch`

## Dependencies (non-Docker)

`sudo apt install ros-melodic-stage ros-melodic-rviz ros-melodic-global-planner ros-melodic-teb-local-planner`
