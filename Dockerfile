# Use the official ROS2 base image base including developing tools
FROM ubuntu:jammy

ENV DEBIAN_FRONTEND=noninteractive \
  LANG=en_US.UTF-8 \
  LC_ALL=en_US.UTF-8

RUN  --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  apt update && apt install -y \
  locales \
  software-properties-common \
  curl \
  && locale-gen en_US en_US.UTF-8 \
  && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

RUN add-apt-repository universe

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" > /etc/apt/sources.list.d/ros2.list

RUN  --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  apt update && apt upgrade -y && \
  apt install -y ros-humble-desktop ros-dev-tools

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

RUN  --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  apt-get update && apt-get install -y \
  iputils-ping \
  net-tools \
  tcpdump \ 
  vim \
  git \
  cmake \
  build-essential \
  python3-pip \
  python3-venv \
  python3-colcon-common-extensions \
  clang \
  lldb \
  ninja-build \
  libgtest-dev \
  libeigen3-dev \
  libopencv-dev \
  libyaml-dev \
  libgstreamer1.0-dev \
  libgstreamer-plugins-base1.0-dev \
  gstreamer1.0-plugins-good \
  gstreamer1.0-tools \
  sudo \
  wget \
  curl \
  tmux \
  ruby \
  tmuxinator

# Install PX4
WORKDIR /root

RUN git clone -b v1.15.4 --depth 1 https://github.com/PX4/PX4-Autopilot.git --recursive

WORKDIR /root/PX4-Autopilot

RUN  --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  bash ./Tools/setup/ubuntu.sh

RUN QT_QPA_PLATFORM=xcb make px4_sitl

# Setup Micro XRCE-DDS Agent & Client
WORKDIR /root 

RUN git clone --depth 1 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git 

WORKDIR /root/Micro-XRCE-DDS-Agent

RUN mkdir build && \
  cd build && \
  cmake .. && \
  make -j 10 && \
  make install && \
  ldconfig /usr/local/lib/

# Build ROS 2 Workspace ws_sensor_combined
WORKDIR /root

RUN mkdir -p /root/ws_sensor_combined/src && \
  cd /root/ws_sensor_combined/src && \
  git clone --depth 1 https://github.com/PX4/px4_msgs.git && \
  git clone --depth 1 https://github.com/PX4/px4_ros_com.git && \
  /bin/bash -c "source /opt/ros/humble/setup.bash && cd /root/ws_sensor_combined && colcon build"

# Build ROS 2 Workspace ws_offboard_control
RUN mkdir -p /root/ws_offboard_control/src && \
  cd /root/ws_offboard_control/src && \
  git clone --depth 1 https://github.com/PX4/px4_msgs.git && \
  git clone --depth 1 https://github.com/PX4/px4_ros_com.git && \
  /bin/bash -c "source /opt/ros/humble/setup.bash && cd /root/ws_offboard_control && colcon build"

# Install Python requirements. If you don't have gpu, uncomment next line -torch cpu installation-
#RUN pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu

RUN pip3 install \
  mavsdk \
  aioconsole \
  pygame \
  opencv-python \
  ultralytics

RUN  --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  apt-get install -y ros-humble-ros-gz

# Related to mismatch between numpy 2.x and numpy 1.x
RUN pip3 uninstall -y numpy

# Copy models and worlds from local repository
RUN mkdir -p /root/.gz/fuel/fuel.ignitionrobotics.org/openrobotics/models/
COPY . /root/PX4-ROS2-Gazebo-YOLOv8
COPY models/. /root/.gz/models/
COPY models_docker/. /root/.gz/fuel/fuel.ignitionrobotics.org/openrobotics/models/
COPY worlds/default_docker.sdf /root/PX4-Autopilot/Tools/simulation/gz/worlds/default.sdf

# Modify camera angle
RUN sed -i 's|<pose>.12 .03 .242 0 0 0</pose>|<pose>.15 .029 .21 0 0.7854 0</pose>|' /root/PX4-Autopilot/Tools/simulation/gz/models/x500_depth/model.sdf

# Additional Configs
RUN echo "source /root/ws_sensor_combined/install/setup.bash" >> /root/.bashrc && \
  echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
  echo "export GZ_SIM_RESOURCE_PATH=/root/.gz/models" >> /root/.bashrc

# Copy tmuxinator configuration
COPY px4_ros2_gazebo.yml /root/.config/tmuxinator/px4_ros2_gazebo.yml

# Set up tmuxinator
RUN echo "export PATH=\$PATH:/root/.local/bin" >> /root/.bashrc

#NOTE: without this, won't be able to publish the image
RUN apt update && apt install ros-humble-ros-gzgarden -y

# Set default command to start tmuxinator
# CMD ["tmuxinator", "start", "px4_ros2_gazebo"]
