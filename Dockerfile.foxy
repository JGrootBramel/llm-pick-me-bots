# Base: Ubuntu 20.04 + ROS 2 Foxy + Gazebo + RViz
FROM osrf/ros:foxy-desktop

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=foxy
ENV USERNAME=ros

# ---------------------------------------------------------
# 1) Basic tools & ROS 2 dev utilities
# ---------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    build-essential \
    cmake \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    wget \
    nano \
    # extra sim tooling
    ros-foxy-gazebo-ros-pkgs \
    ros-foxy-joint-state-publisher-gui \
    ros-foxy-rqt-robot-steering \
    ros-foxy-teleop-twist-keyboard \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep (inside container)
RUN rosdep init || echo "rosdep already initialized" && \
    rosdep update

# ---------------------------------------------------------
# 2) Create non-root user for development
# ---------------------------------------------------------
RUN useradd -ms /bin/bash ${USERNAME} && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME}

USER ${USERNAME}
WORKDIR /home/${USERNAME}

# ---------------------------------------------------------
# 3) Create ROS 2 workspace and clone packages
# ---------------------------------------------------------
RUN mkdir -p /home/${USERNAME}/limo_cobot_ws/src
WORKDIR /home/${USERNAME}/limo_cobot_ws/src

# 3.1 Limo ROS2 (use foxy branch if available; falls back to default)
# NOTE: limo_ros2 upstream describes Humble, but the code is standard ROS 2
# and has a foxy branch in the history. This should build on Foxy.
RUN git clone -b foxy --depth 1 https://github.com/agilexrobotics/limo_ros2.git || \
    git clone --depth 1 https://github.com/agilexrobotics/limo_ros2.git

# 3.2 myCobot ROS2 (explicitly foxy branch)
RUN git clone -b foxy --depth 1 https://github.com/elephantrobotics/mycobot_ros2.git

# 3.3 Limo Cobot sim (ROS1 / catkin) – cloned as REFERENCE, ignored by colcon
RUN git clone --depth 1 https://github.com/agilexrobotics/limo_cobot_sim.git && \
    touch /home/${USERNAME}/limo_cobot_ws/src/limo_cobot_sim/COLCON_IGNORE

# ---------------------------------------------------------
# 4) Install dependencies via rosdep
# ---------------------------------------------------------
WORKDIR /home/${USERNAME}/limo_cobot_ws

# Source Foxy and install package dependencies
RUN /bin/bash -lc "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y || echo 'rosdep missing some keys, continuing'"

# ---------------------------------------------------------
# 5) Build ROS 2 workspace (limo_ros2 + mycobot_ros2)
# ---------------------------------------------------------
RUN /bin/bash -lc "source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build"

# ---------------------------------------------------------
# 6) Convenience: environment setup in .bashrc
# ---------------------------------------------------------
RUN echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> /home/${USERNAME}/.bashrc && \
    echo 'source ~/limo_cobot_ws/install/setup.bash' >> /home/${USERNAME}/.bashrc && \
    echo 'export RMW_IMPLEMENTATION=rmw_fastrtps_cpp' >> /home/${USERNAME}/.bashrc && \
    echo 'export ROS_DOMAIN_ID=7' >> /home/${USERNAME}/.bashrc

# Default workdir when you `docker run`
WORKDIR /home/${USERNAME}/limo_cobot_ws

CMD ["/bin/bash"]