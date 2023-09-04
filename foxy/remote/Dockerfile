# ROS distribution to use
ARG ROS_DISTRO=foxy

FROM osrf/ros:${ROS_DISTRO}-desktop

# Let us install tzdata painlessly
ENV DEBIAN_FRONTEND=noninteractive

# Use the bash shell
SHELL ["/bin/bash", "-c"]

# Install prerequiste packages
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    wget \
    python3-pip \
    python-is-python3 \
    iproute2 \
    neovim \
    locales

# Change the locale from POSIX to UTF-8
RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV PYTHONIOENCODING=utf-8

# Create workspace
RUN mkdir -p /robot_ws
WORKDIR /robot_ws

# Install ROS 2-related and TurtleBot3 packages
RUN apt-get install -y \
    # DDS
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    # Gazebo
    ros-${ROS_DISTRO}-gazebo-* \
    # Cartographer
    ros-${ROS_DISTRO}-cartographer \
    ros-${ROS_DISTRO}-cartographer-ros \
    # Navigation2
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    # TurtleBot3
    ros-${ROS_DISTRO}-dynamixel-sdk \
    ros-${ROS_DISTRO}-turtlebot3-msgs \
    ros-${ROS_DISTRO}-turtlebot3

COPY requirements.txt .

RUN python3 -m pip install --no-deps -r requirements.txt 

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc && \
    echo "source /usr/share/vcstool-completion/vcs.bash" >> ~/.bashrc

# https://answers.ros.org/question/372890/rqt-button-not-show-icon/
RUN mkdir ~/.icons && \
    ln -s /usr/share/icons/Tango ~/.icons/hicolor

RUN echo "alias vi=nvim" >> ~/.bash_aliases

