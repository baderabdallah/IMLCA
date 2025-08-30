# Base ROS Noetic (Ubuntu 20.04) with minimal ROS tools
FROM ros:noetic-ros-base

ENV DEBIAN_FRONTEND=noninteractive \
    TERM=xterm-256color \
    CATKIN_WS=/root/catkin_ws

# System and ROS dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git curl vim rsync \
    python3-pip python3-dev python-is-python3 \
    ros-noetic-catkin ros-noetic-roslaunch \
    ros-noetic-roscpp ros-noetic-rospy ros-noetic-std-msgs \
    ros-noetic-message-generation ros-noetic-message-runtime \
    ros-noetic-rospack \
    # headless & video utils for matplotlib/gym
    ffmpeg xvfb xauth x11-apps \
    libgl1 libglib2.0-0 libsm6 libxext6 libxrender1 \
    libgl1-mesa-dri libglu1-mesa mesa-utils \
    qt5-default python3-pyqt5 \
    # X11/OpenAL libs for Panda3D
    libx11-6 libxrandr2 libxxf86vm1 libxcursor1 libxi6 libxinerama1 libopenal1 \
    && rm -rf /var/lib/apt/lists/*

# Upgrade pip tooling first
RUN pip install --no-cache-dir --upgrade pip setuptools wheel

# Core scientific stack and ROS Python helpers
RUN pip install --no-cache-dir \
    numpy==1.24.4 \
    matplotlib==3.7.5 \
    pandas==1.5.3 \
    pygame==2.5.2 \
    rospkg==1.5.0

# PyTorch CPU wheels (use the CPU index URL)
RUN pip install --no-cache-dir \
    --index-url https://download.pytorch.org/whl/cpu \
    torch==2.2.2 torchvision==0.17.2 torchaudio==2.2.2

# RL ecosystem
RUN pip install --no-cache-dir \
    gymnasium==0.29.1 \
    stable-baselines3==2.2.1 \
    highway-env==1.8.2

# rl-agents (not on PyPI) from GitHub
RUN pip install --no-cache-dir \
    git+https://github.com/eleurent/rl-agents@master

# Panda3D engine for 3D viewer
RUN pip install --no-cache-dir panda3d==1.10.13

# Create workspace layout
RUN mkdir -p ${CATKIN_WS}/src
WORKDIR ${CATKIN_WS}

# Copy entrypoint (located in docker/ relative to build context root)
COPY docker/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

# Default environment
ENV ROS_DISTRO=noetic \
    ROS_VERSION=1 \
    ROS_PYTHON_VERSION=3 \
    HEADLESS=1 \
    LAUNCH_FILE=manual_controller_headless.launch

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["roslaunch"]
