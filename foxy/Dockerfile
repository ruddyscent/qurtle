# ROS distribution to use
ARG ROS_DISTRO=foxy

FROM osrf/ros:${ROS_DISTRO}-desktop

ARG ROS_DISTRO

# Let us install tzdata painlessly
ENV DEBIAN_FRONTEND=noninteractive

# Use the bash shell
SHELL ["/bin/bash", "-c"]

# Create workspace
RUN mkdir -p /turtlebot3_ws

WORKDIR /turtlebot3_ws

# Install prerequiste packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    wget \
    python3-pip \
    python-is-python3

# Install ROS 2
RUN wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros2_foxy.sh && \
    chmod 755 ./install_ros2_foxy.sh && \
    source ./install_ros2_foxy.sh

# Install ROS 2-related and TurtleBot3 packages
RUN apt-get install -y --no-install-recommends \
    # Install Gazebo
    ros-foxy-gazebo-* \
    # Install Cartographer
    ros-foxy-cartographer \
    ros-foxy-cartographer-ros \
    # Install Navigation2
    ros-foxy-navigation2 \
    ros-foxy-nav2-bringup \
    # Install TurtleBot3 Packages
    ros-foxy-dynamixel-sdk \
    ros-foxy-turtlebot3-msgs \
    ros-foxy-turtlebot3

COPY requirements.txt .

RUN python3 -m pip install --no-deps -r requirements.txt 

# Configure environment
RUN echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc && \
    source ~/.bashrc

