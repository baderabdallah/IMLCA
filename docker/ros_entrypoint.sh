#!/usr/bin/env bash
set -e

# Prepare ROS environment
source /opt/ros/$ROS_DISTRO/setup.bash

export CATKIN_WS=${CATKIN_WS:-/root/catkin_ws}
mkdir -p "$CATKIN_WS/src"

# Runtime env
export OMP_NUM_THREADS=1
export ROS_MASTER_URI=${ROS_MASTER_URI:-http://localhost:11311}

# Display/backends
if [ "${HEADLESS:-1}" = "1" ]; then
  export MPLBACKEND=Agg
  export SDL_AUDIODRIVER=dummy
  export SDL_VIDEODRIVER=dummy
else
  # Prefer Qt5Agg for GUI figures
  export MPLBACKEND=${MPLBACKEND:-Qt5Agg}
  # Ensure SDL uses X11 if available
  unset SDL_VIDEODRIVER || true
  # Hint Panda3D to use X11/GL instead of unknown defaults
  export PANDA_FORCE_PARASITE_BUFFER=1
  export PRC_DATA="load-display pandagl\naux-display pandadx9\n"
fi

# If the repo is mounted at /workspace, link it into catkin src
if [ -d /workspace/src ]; then
  echo "Detected /workspace; syncing sources into catkin workspace..."
  rsync -a --delete /workspace/src/ "$CATKIN_WS/src/"
else
  # Fallback: if repo is copied into /root/IMLCA, link it
  if [ -d /root/IMLCA/src ]; then
    rsync -a --delete /root/IMLCA/src/ "$CATKIN_WS/src/"
  fi
fi

# Build
echo "Building catkin workspace at $CATKIN_WS..."
cd "$CATKIN_WS"
catkin_make
source "$CATKIN_WS/devel/setup.bash"

# Launch
if [ -n "$LAUNCH_FILE" ]; then
  echo "Launching: $LAUNCH_FILE (HEADLESS=$HEADLESS)"
  export HEADLESS
  # If LAUNCH_FILE is a bare filename and exists under /workspace, use full path
  if [[ "$LAUNCH_FILE" != *"/"* ]] && [ -f "/workspace/$LAUNCH_FILE" ]; then
    exec roslaunch "/workspace/$LAUNCH_FILE"
  else
    exec roslaunch "$LAUNCH_FILE"
  fi
else
  echo "No LAUNCH_FILE specified; starting bash"
  exec bash
fi
