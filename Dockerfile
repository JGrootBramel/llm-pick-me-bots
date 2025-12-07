# Base: Ubuntu 22.04 + ROS 2 Humble + Gazebo + RViz
FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# ---------------------------------------------------------
# 1) Basic tools and ROS / Gazebo dependencies
# ---------------------------------------------------------
RUN apt-get update && apt-get install -y \
    git \
    python3-pip \
    python3-colcon-common-extensions \
    ros-humble-joint-state-publisher-gui \
    ros-humble-rqt-robot-steering \
    ros-humble-teleop-twist-keyboard \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# Optional: Python libs for your LLM / tooling
RUN pip3 install --no-cache-dir \
    pymycobot \
    openai \
    numpy \
    tqdm

# ---------------------------------------------------------
# 2) Create non-root user
# ---------------------------------------------------------
ARG USERNAME=ros
ARG UID=1000
ARG GID=1000

RUN groupadd --gid $GID $USERNAME && \
    useradd --uid $UID --gid $GID -m $USERNAME && \
    usermod -aG sudo $USERNAME && \
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

USER $USERNAME
ENV HOME=/home/$USERNAME
WORKDIR $HOME

# ---------------------------------------------------------
# 3) Create ROS 2 workspace and clone repos
# ---------------------------------------------------------
RUN mkdir -p $HOME/limo_cobot_ws/src
WORKDIR $HOME/limo_cobot_ws/src

# 3.1 Limo ROS2 (Humble branch)
RUN git clone -b humble https://github.com/agilexrobotics/limo_ros2.git

# IMPORTANT: Create missing directories expected by limo_car CMake
RUN mkdir -p $HOME/limo_cobot_ws/src/limo_ros2/limo_car/log && \
    mkdir -p $HOME/limo_cobot_ws/src/limo_ros2/limo_car/src && \
    mkdir -p $HOME/limo_cobot_ws/src/limo_ros2/limo_car/worlds

# 3.2 MyCobot 280 ROS2 (Humble branch)
RUN git clone -b humble --depth 1 https://github.com/elephantrobotics/mycobot_ros2.git

# (Optional) 3.3 Your own packages can go here later, or you mount them as a volume

# ---------------------------------------------------------
# 4) Build workspace
# ---------------------------------------------------------
WORKDIR $HOME/limo_cobot_ws

# Use bash login shell (-l) so "source" works correctly
RUN /bin/bash -lc "source /opt/ros/$ROS_DISTRO/setup.bash && colcon build"

# ---------------------------------------------------------
# 5) Environment setup for interactive shells
# ---------------------------------------------------------
RUN echo 'source /opt/ros/$ROS_DISTRO/setup.bash' >> $HOME/.bashrc && \
    echo 'source ~/limo_cobot_ws/install/setup.bash' >> $HOME/.bashrc && \
    echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> $HOME/.bashrc

CMD ["bash"]