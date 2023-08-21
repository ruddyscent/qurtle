# ROS distribution to use
ARG ROS_DISTRO=humble

#############################
# Remote PC for TurtleBot 3 #
#############################
FROM osrf/ros:${ROS_DISTRO}-desktop as remote

# ARG ROS_DISTRO

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

# Install ROS 2-related and TurtleBot3 packages
RUN apt-get install -y --no-install-recommends \
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

# Configure environment
RUN echo "export ROS_DOMAIN_ID=30 #TURTLEBOT3" >> ~/.bashrc && \
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

##############################
# SBC using Jetson Xaiver NX #
##############################
FROM dustynv/ros:humble-desktop-l4t-r35.4.1 as sbc

# Let us install tzdata painlessly
ENV DEBIAN_FRONTEND=noninteractive

# Use the bash shell
SHELL ["/bin/bash", "-c"]

# Build and Install TurtleBot 3 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-argcomplete \
    python3-colcon-common-extensions \
    libboost-system-dev \
    build-essential \
    ros-humble-hls-lfcd-lds-driver \
    ros-humble-turtlebot3-msgs \
    ros-humble-dynamixel-sdk \
    libudev-dev

RUN mkdir -p ~/turtlebot3_ws/src && \
    cd ~/turtlebot3_ws/src && \
    git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3.git && \
    git clone -b ros2-devel https://github.com/ROBOTIS-GIT/ld08_driver.git && \
    cd ~/turtlebot3_ws/src/turtlebot3 && \
    rm -r turtlebot3_cartographer turtlebot3_navigation2 && \
    cd ~/turtlebot3_ws/ && \
    colcon build --symlink-install --parallel-workers 1 && \
    echo 'source ~/turtlebot3_ws/install/setup.bash' >> ~/.bashrc

# USB Port Setting for OpenCR
RUN cp `ros2 pkg prefix turtlebot3_bringup`/share/turtlebot3_bringup/script/99-turtlebot3-cdc.rules /etc/udev/rules.d/ && \
    udevadm control --reload-rules && \
    udevadm trigger

# Set ROS Domain ID
RUN 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc

# Configure LDS
RUN 'export LDS_MODEL=LDS-02' >> ~/.bashrc
