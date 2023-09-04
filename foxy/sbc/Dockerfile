# ROS distribution to use
ARG ROS_DISTRO=foxy
ARG JETPACK_VER=r35.4.1

# https://hub.docker.com/r/dustynv/ros
FROM dustynv/ros:${ROS_DISTRO}-ros-base-l4t-${JETPACK_VER}

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

# Change the locale from POSIX to UTF-8
RUN locale-gen en_US en_US.UTF-8 && \
    update_locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV PYTHONIOENCODING=utf-8

# Create workspace
RUN mkdir -p /robot_ws
WORKDIR /robot_ws

# Install ROS 2-related and TurtleBot3 packages
RUN apt-get install -y \
    # DDS
    ros-foxy-hls-lfcd-lds-driver \
    # TurtleBot3
    ros-${ROS_DISTRO}-turtlebot3-msgs \
    ros-${ROS_DISTRO}-dynamixel-sdk \
    ros-${ROS_DISTRO}-turtlebot3

# USB port setting for OpenCR
RUN cp `ros2 pkg prefix turtlebot3_bringup`/share/turtlebot3_bringup/script/99-turtlebot3-cdc.rules /etc/udev/rules.d/ && \
    udevadm control --reload-rules && \
    udevadm trigger

# Install LDS
# RUN cd /turtlebot3_ws/src && \
#     git clone -b ros2-devel https://github.com/ROBOTIS-GIT/ld08_driver.git && \
#     cd ~/turtlebot3_ws && colcon build --symlink-install

COPY requirements.txt .

RUN python3 -m pip install --no-deps -r requirements.txt

RUN echo "alias vi=nvim" >> ~/.bash_aliases

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc # &&
#    echo 'source /turtlebot3_ws/install/setup.bash' >> ~/.bashrc
