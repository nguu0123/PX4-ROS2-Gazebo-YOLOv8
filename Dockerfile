# Use the official ROS2 base image base including developing tools
FROM ros:humble-ros-base-jammy
#FROM osrf/ros:humble-desktop-full

# Install necessary packages
RUN rm /etc/apt/sources.list.d/ros2-latest.list && \
  rm /usr/share/keyrings/ros2-latest-archive-keyring.gpg

RUN apt update && apt install -y curl ca-certificates 

RUN export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') ;\
  curl -L -s -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" \
  && apt-get update \
  && apt-get install /tmp/ros2-apt-source.deb \
  && rm -f /tmp/ros2-apt-source.deb
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
  bash ./Tools/setup/ubuntu.sh && \
  make px4_sitl

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
RUN pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu

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

# Set default command to start tmuxinator
CMD ["tmuxinator", "start", "px4_ros2_gazebo"]
