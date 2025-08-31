#!/usr/bin/env bash
set -euo pipefail

# Simple launcher for macOS GUI via Docker + XQuartz
# Requirements (macOS):
#  - Docker Desktop installed and running
#  - XQuartz installed (https://www.xquartz.org) — this script will start/configure it automatically

# Allow overrides from environment
: "${LAUNCH_FILE:=manual_controller_visuals.launch}"

# Start and configure XQuartz automatically (macOS only)
if [[ "$(uname -s)" == "Darwin" ]]; then
	# Ensure XQuartz allows network clients and IGLX
	defaults write org.xquartz.X11 nolisten_tcp -bool false || true
	defaults write org.xquartz.X11 enable_iglx -bool true || true

	# Start XQuartz if not running
	if ! pgrep -x "XQuartz" >/dev/null 2>&1; then
		open -g -a XQuartz || true
	fi

	# Wait until X server is listening on port 6000
	for _ in {1..40}; do
		if command -v nc >/dev/null 2>&1; then
			if nc -z 127.0.0.1 6000 >/dev/null 2>&1; then break; fi
		else
			# Fallback: check socket existence
			if ls /private/tmp/com.apple.launchd.*/*:0 >/dev/null 2>&1; then break; fi
		fi
		sleep 0.25
	done

	# Non-interactive xhost permissions for the chosen DISPLAY targets
	# Always allow localhost
	DISPLAY=:0 /opt/X11/bin/xhost + 127.0.0.1 >/dev/null 2>&1 || true
fi

# Prefer a reachable TCP DISPLAY for containers
# 1) Try Mac LAN IP (en0, then en1) => <ip>:0
# 2) Fallback to host.docker.internal:0
if [[ "${DISPLAY:-}" == /private/tmp/* ]] || [[ -z "${DISPLAY:-}" ]]; then
	MAC_IP="$(ipconfig getifaddr en0 2>/dev/null || true)"
	if [[ -z "$MAC_IP" ]]; then
		MAC_IP="$(ipconfig getifaddr en1 2>/dev/null || true)"
	fi
	if [[ -n "$MAC_IP" ]]; then
		export DISPLAY="${MAC_IP}:0"
		# Allow this IP without requiring user to open XQuartz terminal
		if [[ "$(uname -s)" == "Darwin" ]]; then
			DISPLAY=:0 /opt/X11/bin/xhost + "${MAC_IP}" >/dev/null 2>&1 || true
		fi
	else
		export DISPLAY="host.docker.internal:0"
		# As a fallback, allow localhost (already added) which resolves for most setups
	fi
fi
export LAUNCH_FILE

echo "Using DISPLAY=$DISPLAY"
echo "LAUNCH_FILE=$LAUNCH_FILE (GUI mode)"

# Apple Silicon note: uncomment to force AMD64 build if needed
# export DOCKER_DEFAULT_PLATFORM=linux/amd64

# Validate compose, then run
docker compose -f docker/docker-compose.yml config >/dev/null
exec docker compose -f docker/docker-compose.yml up --build
