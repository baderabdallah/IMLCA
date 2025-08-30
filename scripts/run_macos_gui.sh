#!/usr/bin/env bash
set -euo pipefail

# Simple launcher for macOS GUI via Docker + XQuartz
# Requirements (macOS):
#  - Docker Desktop installed and running
#  - XQuartz installed (https://www.xquartz.org)
#  - In XQuartz Preferences > Security: enable "Allow connections from network clients"
#  - In an XQuartz terminal: run `xhost + 127.0.0.1` (or `xhost +localhost`)

# Allow overrides from environment
: "${LAUNCH_FILE:=manual_controller_visuals.launch}"
: "${HEADLESS:=0}"

# Prefer host.docker.internal for XQuartz TCP display on macOS
# If DISPLAY points to a local socket path, override to TCP form
if [[ "${DISPLAY:-}" == /private/tmp/* ]] || [[ -z "${DISPLAY:-}" ]]; then
	export DISPLAY="host.docker.internal:0"
fi
export LAUNCH_FILE
export HEADLESS

echo "Using DISPLAY=$DISPLAY"
echo "LAUNCH_FILE=$LAUNCH_FILE (HEADLESS=$HEADLESS)"

# Apple Silicon note: uncomment to force AMD64 build if needed
# export DOCKER_DEFAULT_PLATFORM=linux/amd64

# Validate compose, then run
docker compose -f docker/docker-compose.yml config >/dev/null
exec docker compose -f docker/docker-compose.yml up --build
