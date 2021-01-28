FROM ros:melodic

# non-root user info
ARG USERNAME=guest
ARG UID=1000
ARG GID=$UID

# dependencies
ARG PKGS="sudo gdb \
	ros-$ROS_DISTRO-stage ros-$ROS_DISTRO-stage-ros \
	ros-$ROS_DISTRO-rqt ros-$ROS_DISTRO-rqt-common-plugins \
	ros-$ROS_DISTRO-global-planner ros-$ROS_DISTRO-teb-local-planner \
	ros-$ROS_DISTRO-move-base ros-$ROS_DISTRO-amcl ros-$ROS_DISTRO-rviz"

# install dependencies and clean up afterwards
RUN apt-get update \
	&& apt-get -y install --no-install-recommends $PKGS \
	&& apt-get -y autoremove \
	&& apt-get -y clean \
	&& rm -rf /var/lib/apt/lists/*

# adapted from https://code.visualstudio.com/docs/remote/containers-advanced
RUN groupadd --gid $GID $USERNAME \
	&& useradd --uid $UID --gid $GID --create-home --shell /bin/bash $USERNAME \
	&& echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
	&& chmod 0440 /etc/sudoers.d/$USERNAME

# swap to non-root user
USER $USERNAME
WORKDIR /home/$USERNAME

# auto source ros stuff
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
