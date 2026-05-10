ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}

ARG DEBIAN_FRONTEND=noninteractive
ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=${USER_UID}
ARG LIBFRANKA_VERSION=0.9.2
ARG MUJOCO_VERSION=3.2.0
ARG MULTIPANDA_REF=humble
ARG MULTIPANDA_REMOTE=https://github.com/tenfoldpaper/multipanda_ros2.git

ARG COLCON_BUILD_MODE=parallel
ARG COLCON_PARALLEL_WORKERS=
ARG CMAKE_BUILD_PARALLEL_LEVEL=
ARG MAKEFLAGS=
ARG ROS2_WS_CMAKE_BUILD_TYPE=RelWithDebInfo

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ENV COLCON_BUILD_MODE=${COLCON_BUILD_MODE}
ENV COLCON_PARALLEL_WORKERS=${COLCON_PARALLEL_WORKERS}
ENV CMAKE_BUILD_PARALLEL_LEVEL=${CMAKE_BUILD_PARALLEL_LEVEL}
ENV MAKEFLAGS=${MAKEFLAGS}
ENV ROS2_WS_CMAKE_BUILD_TYPE=${ROS2_WS_CMAKE_BUILD_TYPE}

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-desktop \
    sudo \
    git \
    curl \
    ca-certificates \
    software-properties-common \
    clang-14 \
    clang-format-14 \
    clang-tidy-14 \
    build-essential \
    cmake \
    ninja-build \
    pkg-config \
    gdb \
    python3-pip \
    python3-rosdep \
    python3-vcstool \
    python3-colcon-common-extensions \
    python3-argcomplete \
    python3-dev \
    python3-setuptools \
    libboost-all-dev \
    libfmt-dev \
    libpoco-dev \
    libyaml-cpp-dev \
    liborocos-kdl-dev \
    libtinyxml2-dev \
    libxml2-dev \
    libqt5gui5 \
    libqt5widgets5 \
    libqt5core5a \
    libglfw3 \
    libglfw3-dev \
    libglew-dev \
    libgl1-mesa-dev \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    novnc \
    websockify \
    xvfb \
    x11vnc \
    x11-utils \
    openbox \
    libxrandr-dev \
    libxinerama-dev \
    libxcursor-dev \
    libxi-dev \
    wget \
    unzip \
    ros-humble-control-msgs \
    ros-humble-xacro \
    ros-humble-ament-cmake-clang-format \
    ros-humble-ament-clang-format \
    ros-humble-ament-flake8 \
    ros-humble-ament-cmake-clang-tidy \
    ros-humble-angles \
    ros-humble-ros2-control \
    ros-humble-realtime-tools \
    ros-humble-control-toolbox \
    ros-humble-controller-manager \
    ros-humble-hardware-interface \
    ros-humble-hardware-interface-testing \
    ros-humble-launch-testing \
    ros-humble-generate-parameter-library \
    ros-humble-controller-interface \
    ros-humble-ros2-control-test-assets \
    ros-humble-moveit \
    ros-humble-nav-msgs \
    ros-humble-joint-state-publisher-gui \
    ros-humble-camera-info-manager \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-broadcaster \
    ros-humble-warehouse-ros-sqlite \
    ros-humble-joint-trajectory-controller \
    ros-humble-rqt \
    ros-humble-rqt-common-plugins \
    ros-humble-rqt-controller-manager \
    ros-humble-rqt-robot-steering \
    ros-humble-rqt-tf-tree \
    ros-humble-rqt-graph \
    ros-humble-rqt-reconfigure \
    ros-humble-rqt-plot \
    ros-humble-teleop-twist-keyboard \
    ros-humble-joy \
    ros-humble-teleop-twist-joy \
 && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --no-cache-dir \
    numpy \
    viser \
    yourdfpy

RUN mkdir -p /tmp/.X11-unix \
 && chown root:root /tmp/.X11-unix \
 && chmod 1777 /tmp/.X11-unix

RUN if ! getent group "${USER_GID}" >/dev/null 2>&1; then \
      groupadd --gid "${USER_GID}" "${USERNAME}"; \
    fi \
 && if ! id -u "${USERNAME}" >/dev/null 2>&1; then \
      useradd --uid "${USER_UID}" --gid "${USER_GID}" --shell /bin/bash -m "${USERNAME}"; \
    fi \
 && usermod -aG sudo "${USERNAME}" \
 && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > "/etc/sudoers.d/${USERNAME}" \
 && chmod 0440 "/etc/sudoers.d/${USERNAME}"

RUN rosdep init 2>/dev/null || true

RUN git clone --depth 1 --branch 3.3.9 https://gitlab.com/libeigen/eigen.git /tmp/eigen \
 && cmake -S /tmp/eigen -B /tmp/eigen/build \
 && cmake --install /tmp/eigen/build \
 && rm -rf /tmp/eigen

RUN arch="$(dpkg --print-architecture)" \
 && if [ "${arch}" = "amd64" ]; then \
      add-apt-repository -y ppa:dqrobotics-dev/release \
      && apt-get update \
      && apt-get install -y --no-install-recommends libdqrobotics; \
    elif [ "${arch}" = "arm64" ]; then \
      git clone --depth 1 https://github.com/dqrobotics/cpp.git /tmp/dqrobotics-cpp \
      && cmake -S /tmp/dqrobotics-cpp -B /tmp/dqrobotics-cpp/build -DCMAKE_BUILD_TYPE=Release \
      && dq_build_args=() \
      && if [ -n "${CMAKE_BUILD_PARALLEL_LEVEL}" ]; then \
           dq_build_args+=(--parallel "${CMAKE_BUILD_PARALLEL_LEVEL}"); \
         fi \
      && cmake --build /tmp/dqrobotics-cpp/build "${dq_build_args[@]}" \
      && cmake --install /tmp/dqrobotics-cpp/build \
      && rm -rf /tmp/dqrobotics-cpp \
      && ldconfig; \
    else \
      echo "Unsupported architecture for DQ Robotics: ${arch}" >&2; \
      exit 1; \
    fi \
 && rm -rf /var/lib/apt/lists/*

RUN git clone --depth 1 --recurse-submodules --branch "${LIBFRANKA_VERSION}" https://github.com/frankaemika/libfranka.git /tmp/libfranka \
 && cmake -S /tmp/libfranka -B /tmp/libfranka/build -DBUILD_TESTS=OFF -DCMAKE_BUILD_TYPE=Release \
 && libfranka_build_args=() \
 && if [ -n "${CMAKE_BUILD_PARALLEL_LEVEL}" ]; then \
      libfranka_build_args+=(--parallel "${CMAKE_BUILD_PARALLEL_LEVEL}"); \
    fi \
 && cmake --build /tmp/libfranka/build "${libfranka_build_args[@]}" \
 && cmake --install /tmp/libfranka/build \
 && rm -rf /tmp/libfranka

RUN arch="$(dpkg --print-architecture)" \
 && case "${arch}" in \
      amd64) mujoco_arch="linux-x86_64" ;; \
      arm64) mujoco_arch="linux-aarch64" ;; \
      *) echo "Unsupported architecture: ${arch}" >&2; exit 1 ;; \
    esac \
 && curl -fsSL "https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}/mujoco-${MUJOCO_VERSION}-${mujoco_arch}.tar.gz" -o /tmp/mujoco.tgz \
 && mkdir -p /opt/mujoco \
 && tar -xzf /tmp/mujoco.tgz -C /opt/mujoco --strip-components=1 \
 && mkdir -p /opt/mujoco/lib/cmake/mujoco \
 && printf '%s\n' \
    'get_filename_component(_mujoco_prefix "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)' \
    '' \
    'if(NOT TARGET mujoco::mujoco)' \
    '  add_library(mujoco::mujoco SHARED IMPORTED)' \
    '  set_target_properties(mujoco::mujoco PROPERTIES' \
    '    IMPORTED_LOCATION "${_mujoco_prefix}/lib/libmujoco.so"' \
    '    INTERFACE_INCLUDE_DIRECTORIES "${_mujoco_prefix}/include"' \
    '  )' \
    'endif()' \
    '' \
    'set(mujoco_FOUND TRUE)' \
    'set(mujoco_INCLUDE_DIRS "${_mujoco_prefix}/include")' \
    'set(mujoco_LIBRARIES mujoco::mujoco)' \
    > /opt/mujoco/lib/cmake/mujoco/mujocoConfig.cmake \
 && printf '%s\n' \
    'set(PACKAGE_VERSION "3.2.0")' \
    '' \
    'if(PACKAGE_FIND_VERSION VERSION_GREATER PACKAGE_VERSION)' \
    '  set(PACKAGE_VERSION_COMPATIBLE FALSE)' \
    'else()' \
    '  set(PACKAGE_VERSION_COMPATIBLE TRUE)' \
    '  if(PACKAGE_FIND_VERSION VERSION_EQUAL PACKAGE_VERSION)' \
    '    set(PACKAGE_VERSION_EXACT TRUE)' \
    '  endif()' \
    'endif()' \
    > /opt/mujoco/lib/cmake/mujoco/mujocoConfigVersion.cmake \
 && rm -f /tmp/mujoco.tgz \
 && ldconfig

ENV ROS_DISTRO=${ROS_DISTRO}
ENV MUJOCO_DIR=/opt/mujoco
ENV mujoco_DIR=/opt/mujoco/lib/cmake/mujoco
ENV CMAKE_PREFIX_PATH=/opt/mujoco:/opt/mujoco/lib/cmake:/usr/local/lib/cmake:/usr/local/share/eigen3/cmake
ENV LD_LIBRARY_PATH=/opt/mujoco/lib:/usr/local/lib:${LD_LIBRARY_PATH}

COPY .devcontainer/patches /tmp/devcontainer-patches

RUN mkdir -p /opt/ros2_ws/src \
 && git clone --recursive --branch "${MULTIPANDA_REF}" "${MULTIPANDA_REMOTE}" /opt/ros2_ws/src/multipanda_ros2 \
 && git -C /opt/ros2_ws/src/multipanda_ros2 apply /tmp/devcontainer-patches/mujoco_ros-headless-executor.patch \
 && rm -rf /opt/ros2_ws/src/multipanda_ros2/franka_example_controllers \
 && cp -a /tmp/devcontainer-patches/multipanda_ros2/franka_example_controllers /opt/ros2_ws/src/multipanda_ros2/ \
 && source "/opt/ros/${ROS_DISTRO}/setup.bash" \
 && cd /opt/ros2_ws \
 && rosdep update \
 && apt-get update \
 && rosdep install \
      --from-paths src \
      --ignore-src \
      --rosdistro "${ROS_DISTRO}" \
      -r \
      -y \
      --skip-keys libfranka \
 && build_args=() \
 && if [ "${COLCON_BUILD_MODE}" = "queued" ]; then \
      build_args+=(--executor sequential); \
    elif [ "${COLCON_BUILD_MODE}" != "parallel" ]; then \
      echo "Unsupported COLCON_BUILD_MODE: ${COLCON_BUILD_MODE}" >&2; \
      echo "Supported values are: parallel, queued" >&2; \
      exit 1; \
    fi \
 && if [ -n "${COLCON_PARALLEL_WORKERS}" ]; then \
      build_args+=(--parallel-workers "${COLCON_PARALLEL_WORKERS}"); \
    fi \
 && colcon build "${build_args[@]}" \
      --cmake-args -DCMAKE_BUILD_TYPE="${ROS2_WS_CMAKE_BUILD_TYPE}" \
 && rm -rf /var/lib/apt/lists/*

COPY .devcontainer/scripts/start-novnc.sh /usr/local/bin/start-novnc

RUN chmod 0755 /usr/local/bin/start-novnc

RUN cat <<EOF >> /etc/bash.bashrc
source /opt/ros/${ROS_DISTRO}/setup.bash
if [ -f /opt/ros2_ws/install/setup.bash ]; then source /opt/ros2_ws/install/setup.bash; fi
if [ -f /rnm-workspace/ros2_ws/install/setup.bash ]; then source /rnm-workspace/ros2_ws/install/setup.bash; fi
start_novnc() {
  export DISPLAY="\${NOVNC_DISPLAY:-:99}"
  /usr/local/bin/start-novnc "\$@"
}
alias novnc='start_novnc'
EOF

WORKDIR /rnm-workspace
