ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base

ARG ROS_DISTRO
ENV ROS_DISTRO=${ROS_DISTRO}
ENV DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-c"]

# ---------------------------------------------------------------------------
# Base system tools + GTSAM build dependencies
# ---------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    sudo \
    python3-pip \
    python3-dev \
    python3-colcon-common-extensions \
    python3-vcstool \
    python3-rosdep \
    libboost-all-dev \
    libeigen3-dev \
    libtbb-dev \
    libmetis-dev \
    libgoogle-glog-dev \
    libgflags-dev \
    libsuitesparse-dev \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# ROS 2 packages used by the estimator node and its dependencies.
# ---------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-rclcpp \
    ros-${ROS_DISTRO}-std-msgs \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-nav-msgs \
    ros-${ROS_DISTRO}-geometry-msgs \
    ros-${ROS_DISTRO}-visualization-msgs \
    ros-${ROS_DISTRO}-tf2 \
    ros-${ROS_DISTRO}-tf2-geometry-msgs \
    ros-${ROS_DISTRO}-rosidl-default-generators \
    ros-${ROS_DISTRO}-rosidl-default-runtime \
    ros-${ROS_DISTRO}-pluginlib \
    ros-${ROS_DISTRO}-rcpputils \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-pinocchio \
    ros-${ROS_DISTRO}-ros2bag \
    ros-${ROS_DISTRO}-rosbag2 \
    ros-${ROS_DISTRO}-rosbag2-transport \
    ros-${ROS_DISTRO}-rosbag2-storage-mcap \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init 2>/dev/null || true && rosdep update --rosdistro ${ROS_DISTRO}

# ---------------------------------------------------------------------------
# Build and install GTSAM from source.
#
# The README documents this package as tested against a source build of the
# `develop` branch (GTSAM_VERSION_STRING "4.3a1"), with GTSAM_BUILD_UNSTABLE
# enabled. No custom GTSAM fork is required: the estimator's GTSAM-derived
# code (LeggedEstimator.{h,cpp}, LeggedEstimatorFactors.h) is vendored inside
# this package's own src/include tree and links against a stock GTSAM
# install. Pin GTSAM_GIT_REF at build time for reproducible rebuilds.
# ---------------------------------------------------------------------------
ARG GTSAM_GIT_REF=develop

WORKDIR /opt
RUN git clone https://github.com/borglab/gtsam.git gtsam && \
    git -C gtsam checkout --detach "${GTSAM_GIT_REF}"

WORKDIR /opt/gtsam
RUN mkdir -p build && cd build && \
    cmake .. \
      -DCMAKE_BUILD_TYPE=Release \
      -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
      -DGTSAM_BUILD_TESTS=OFF \
      -DGTSAM_BUILD_UNSTABLE=ON \
      -DGTSAM_USE_SYSTEM_EIGEN=ON \
      -DCMAKE_INSTALL_PREFIX=/usr/local && \
    make -j"$(nproc)" && \
    make install && \
    ldconfig

# ---------------------------------------------------------------------------
# Trajectory evaluation (APE/RPE), matching tools/z_rpe_evaluator.py.
# ---------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-tk \
    && rm -rf /var/lib/apt/lists/*
RUN python3 -m pip install --break-system-packages --no-cache-dir evo==1.37.0

# ---------------------------------------------------------------------------
# Non-root development user.
# ---------------------------------------------------------------------------
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=1000

RUN if ! getent group ${USER_GID} >/dev/null; then \
      groupadd --gid ${USER_GID} ${USERNAME}; \
    fi && \
    if ! id -u ${USERNAME} >/dev/null 2>&1; then \
      useradd --uid ${USER_UID} --gid ${USER_GID} --create-home --shell /bin/bash ${USERNAME}; \
    fi && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} && \
    chmod 0440 /etc/sudoers.d/${USERNAME}

WORKDIR /ros2_ws
RUN mkdir -p /ros2_ws/src && chown -R ${USER_UID}:${USER_GID} /ros2_ws

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${USERNAME}/.bashrc && \
    echo "if [ -f /ros2_ws/install/setup.bash ]; then source /ros2_ws/install/setup.bash; fi" >> /home/${USERNAME}/.bashrc && \
    chown ${USER_UID}:${USER_GID} /home/${USERNAME}/.bashrc

USER ${USERNAME}
ENV HOME=/home/${USERNAME}

CMD ["/bin/bash"]
