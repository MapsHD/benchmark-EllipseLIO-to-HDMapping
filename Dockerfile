FROM ubuntu:22.04

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# ── Base tools ────────────────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common \
    build-essential \
    git \
    apt-transport-https \
    ca-certificates \
    wget \
    libeigen3-dev \
    libboost-all-dev \
    libomp-dev \
    libpcl-dev \
    libopencv-dev \
    libyaml-cpp-dev \
    nlohmann-json3-dev \
    tmux \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# ── ROS 2 Humble ─────────────────────────────────────────────────────────────
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-desktop \
    ros-humble-tf2-ros \
    ros-humble-tf2-eigen \
    ros-humble-pcl-conversions \
    ros-humble-pcl-ros \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-message-filters \
    ros-humble-geometry-msgs \
    ros-humble-nav-msgs \
    ros-humble-sensor-msgs \
    ros-humble-std-srvs \
    ros-humble-visualization-msgs \
    ros-humble-rosbag2-cpp \
    ros-humble-rosbag2-storage \
    ros-humble-rosbag2-storage-default-plugins \
    ros-humble-rclcpp-components \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# ── rosbags (Python tool to convert ROS 1 bags to ROS 2 format) ──────────────
RUN pip3 install --no-cache-dir "rosbags==0.9.22"

# ── Build colcon workspace ────────────────────────────────────────────────────
WORKDIR /ros2_ws

COPY ./src/ellipselio                ./src/ellipselio
COPY ./src/ellipselio-to-hdmapping   ./src/ellipselio-to-hdmapping

# Fetch LASzip at the same commit used by the other benchmarks in this repo
# (master introduced a CMAKE_SOURCE_DIR bug in dll/CMakeLists.txt that breaks
# subdirectory builds; this pinned commit predates that change)
RUN git clone https://github.com/LASzip/LASzip.git \
    src/ellipselio-to-hdmapping/src/3rdparty/LASzip && \
    git -C src/ellipselio-to-hdmapping/src/3rdparty/LASzip checkout 4aada84

RUN source /opt/ros/humble/setup.bash && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# ── Non-root user ─────────────────────────────────────────────────────────────
ARG UID=1000
ARG GID=1000
RUN groupadd -g $GID ros && \
    useradd -m -u $UID -g $GID -s /bin/bash ros && \
    chown -R $UID:$GID /ros2_ws

RUN echo "source /opt/ros/humble/setup.bash"    >> /root/.bashrc && \
    echo "source /ros2_ws/install/setup.bash"    >> /root/.bashrc && \
    echo "source /opt/ros/humble/setup.bash"    >> /home/ros/.bashrc && \
    echo "source /ros2_ws/install/setup.bash"    >> /home/ros/.bashrc

CMD ["bash"]
