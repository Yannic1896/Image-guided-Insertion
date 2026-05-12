ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-ros-base

ARG DEBIAN_FRONTEND=noninteractive
ARG USERNAME=rosdev
ARG USER_UID=1000
ARG USER_GID=${USER_UID}

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Core build tools + colcon + common ROS 2 Python utilities
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-argcomplete \
    python3-pip \
    python3-setuptools \
    python3-pytest \
    ros-${ROS_DISTRO}-launch \
    ros-${ROS_DISTRO}-launch-ros \
    ros-${ROS_DISTRO}-ros2launch \
    ros-${ROS_DISTRO}-std-msgs \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-geometry-msgs \
    ros-${ROS_DISTRO}-nav-msgs \
    ros-${ROS_DISTRO}-rclpy \
    sudo \
    git \
    curl \
    wget \
    vim \
 && rm -rf /var/lib/apt/lists/*

# Non-root user matching the host UID so bind-mounted files are writable
RUN if ! getent group "${USER_GID}" >/dev/null 2>&1; then \
      groupadd --gid "${USER_GID}" "${USERNAME}"; \
    fi \
 && if ! id -u "${USERNAME}" >/dev/null 2>&1; then \
      useradd --uid "${USER_UID}" --gid "${USER_GID}" --shell /bin/bash -m "${USERNAME}"; \
    fi \
 && usermod -aG sudo "${USERNAME}" \
 && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > "/etc/sudoers.d/${USERNAME}" \
 && chmod 0440 "/etc/sudoers.d/${USERNAME}"

# Initialise rosdep
RUN rosdep init 2>/dev/null || true

USER ${USERNAME}
RUN rosdep update

# Pre-source ROS 2 in every interactive shell
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc \
 && echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.profile

WORKDIR /ros2_ws

COPY --chown=${USERNAME}:${USERNAME} docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
