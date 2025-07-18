# Base image 
FROM osrf/ros:noetic-desktop-full


# Setup build args
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=1000
ENV TURTLEBOT3_MODEL=burger

# Install basic utilities
RUN apt-get update && apt-get install -y \
    sudo \
    nano \
    git \
    python3-pip \
    python3-catkin-tools \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*


# Install TurtleBot3 packages
RUN apt-get update && apt-get install -y \
    ros-noetic-turtlebot3 \
    ros-noetic-turtlebot3-simulations

# Set TurtleBot3 model environment variable
RUN echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc

# Create a non-root user
RUN groupadd --gid ${USER_GID} ${USERNAME} && \
    useradd --uid ${USER_UID} --gid ${USER_GID} --create-home --shell /bin/bash ${USERNAME} && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers

# Switch to non-root user
USER ${USERNAME}
WORKDIR /home/${USERNAME}

# Explicitly create workspace using the path (not the env var)
RUN mkdir -p /home/${USERNAME}/catkin_ws/src
RUN git clone https://github.com/JosiahSBern/emotion-recognition-robot.git

# Source ROS environments automatically
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
    echo "source /home/${USERNAME}/catkin_ws/devel/setup.bash" >> ~/.bashrc

# Final working directory
WORKDIR /home/${USERNAME}/catkin_ws
