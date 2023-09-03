# ROS distribution to use
ARG ROS_DISTRO=foxy

FROM osrf/ros:${ROS_DISTRO}-desktop

ARG ROS_DISTRO

# Let us install tzdata painlessly
ENV DEBIAN_FRONTEND=noninteractive

# Use the bash shell
SHELL ["/bin/bash", "-c"]

# Install prerequiste packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    wget \
    python3-pip \
    python-is-python3 \
    iproute2 \
    neovim \
    locales

# change the locale from POSIX to UTF-8
RUN locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV PYTHONIOENCODING=utf-8

# Create workspace
RUN mkdir -p /robot_ws

WORKDIR /robot_ws

# Install ROS 2
# RUN wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros2_${ROS_DISTRO}.sh && \
#     chmod 755 ./install_ros2_${ROS_DISTRO}.sh && \
#     source ./install_ros2_${ROS_DISTRO}.sh

# Install ROS 2-related and TurtleBot3 packages
RUN apt-get install -y \
    # DDS
    ros-foxy-rmw-cyclonedds-cpp \
    # Install Gazebo
    ros-${ROS_DISTRO}-gazebo-* \
    # Install Cartographer
    ros-${ROS_DISTRO}-cartographer \
    ros-${ROS_DISTRO}-cartographer-ros \
    # Install Navigation2
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    # Install TurtleBot3 Packages
    ros-${ROS_DISTRO}-dynamixel-sdk \
    ros-${ROS_DISTRO}-turtlebot3-msgs \
    ros-${ROS_DISTRO}-turtlebot3

COPY requirements.txt .

RUN python3 -m pip install --no-deps -r requirements.txt 

# Set the default DDS middleware to cyclonedds
# https://github.com/ros2/rclcpp/issues/1335
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
