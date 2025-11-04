## DEPRECATED: This Dockerfile mirrors .devcontainer/Dockerfile for compose builds.
## Using ros:noetic-ros-base for wider platform support (desktop-full not available on arm64).
FROM ros:noetic-ros-base

ENV DEBIAN_FRONTEND=noninteractive \
    TERM=xterm-256color \
    CATKIN_WS=/root/catkin_ws

# System and ROS dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git curl vim rsync \
    python3-pip python3-dev python-is-python3 \
    python3-rosdep python3-rosinstall python3-vcstools \
    ros-noetic-catkin ros-noetic-roslaunch \
    ros-noetic-roscpp ros-noetic-rospy ros-noetic-std-msgs \
    ros-noetic-message-generation ros-noetic-message-runtime \
    ros-noetic-rospack \
    # video utils for matplotlib/gym
    ffmpeg xvfb xauth x11-apps \
    # Mesa/OpenGL libraries for software rendering
    libgl1-mesa-glx libgl1-mesa-dri libglu1-mesa mesa-utils \
    libgl1 libglib2.0-0 libsm6 libxext6 libxrender1 \
    qt5-default python3-pyqt5 \
    # X11/OpenAL libs for Panda3D
    libx11-6 libxrandr2 libxxf86vm1 libxcursor1 libxi6 libxinerama1 libopenal1 \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep for dependency management
RUN rosdep init || true && rosdep update

# Force software rendering for compatibility in containerized environments
ENV LIBGL_ALWAYS_SOFTWARE=true

# Upgrade pip tooling first
RUN pip install --no-cache-dir --upgrade pip setuptools wheel

# Core scientific stack and ROS Python helpers
# Python in ROS Noetic images is 3.8; NumPy 2.x requires >=3.9, so we stay on NumPy 1.24.x and add runtime shims in nodes.
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

# Panda3D engine and glTF loader for 3D viewer
RUN pip install --no-cache-dir \
    panda3d==1.10.15 \
    panda3d-gltf

# Create workspace layout
RUN mkdir -p ${CATKIN_WS}/src
WORKDIR ${CATKIN_WS}

# Copy entrypoint (located in docker/ relative to build context root)
COPY .devcontainer/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

# Default environment
ENV ROS_MASTER_URI=http://localhost:11311
ENV ROS_HOSTNAME=localhost
ENV LAUNCH_FILE=manual_controller_visuals.launch

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["roslaunch"]
