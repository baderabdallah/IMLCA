# macOS GUI (Docker +## What didn't work (failed attempts)

- Relying on the default macOS `$DISPLAY` exported by XQuartz (`/private/tmp/com.apple.launchd...:0`).
- Mounting `/tmp/.X11-unix` into the container on macOS (not needed; TCP X forwarding via XQuartz is the path).
- Broken `docker-compose.yml` with mis-indented `- LIBGL_ALWAYS_INDIRECT=1` entries (YAML parse error: "line 2: did not find expected key").tz) — Troubleshooting Playbook

This note documents the Panda3D window failure we hit on macOS when running the project with GUI inside Docker, the false starts, and the reliable fix. Point any future assistant here when this recurs.

## Symptom

- Panda3D failed to create a window; the `scene_viewer` node crashed:

```
Exception: Could not open window.
[scene_viewer-5] process has died [pid NNN, exit code 1, ...]
```

## Likely root causes (macOS specifics)

- DISPLAY pointed to XQuartz’s launchd socket path (e.g. `/private/tmp/com.apple.launchd...:0`) which isn’t reachable from Linux containers.
- Docker Compose YAML errors caused the stack to fail before we even ran (e.g., malformed `environment:` list).
- XQuartz not configured to allow network clients (`xhost`/Security pref was missing).
- OpenGL/GLX mismatch in the container; needed software rendering fallbacks for XQuartz.
- Optional: missing `panda3d-gltf` kept 3D models from loading, even after fixing display.

## What didn’t work (failed attempts)

- Relying on the default macOS `$DISPLAY` exported by XQuartz (`/private/tmp/com.apple.launchd...:0`).
- Mounting `/tmp/.X11-unix` into the container on macOS (not needed; TCP X forwarding via XQuartz is the path).
- Running with HEADLESS=1 but expecting GUI windows to appear.
- Broken `docker-compose.yml` with mis-indented `- LIBGL_ALWAYS_INDIRECT=1` entries (YAML parse error: “line 2: did not find expected key”).

## The fix that worked (reliable path)

1) Configure XQuartz
- XQuartz > Preferences > Security: enable “Allow connections from network clients”.
- Restart XQuartz. In an XQuartz terminal:
  - `xhost + 127.0.0.1`

2) Use a TCP DISPLAY that containers can reach
- Force `DISPLAY=host.docker.internal:0` for macOS in the launcher. Our `scripts/run_macos_gui.sh` now does this automatically if `$DISPLAY` looks like a local launchd path.

3) Clean, valid Docker Compose YAML
- `docker/docker-compose.yml` uses an environment mapping (key: value) and avoids the `/tmp/.X11-unix` bind on macOS.

4) Software GL fallbacks in container
- Env vars in compose: `LIBGL_ALWAYS_INDIRECT=1`, `MESA_LOADER_DRIVER_OVERRIDE=llvmpipe`, `GALLIUM_DRIVER=llvmpipe`.
- `ros_entrypoint.sh` prefers Qt5Agg for matplotlib and hints Panda3D to use GL/X.

5) Ensure 3D assets load
- Docker image installs `panda3d-gltf` so `.glb` models in `scene_viewer` load correctly.

6) Launch
- From repo root:
  - `scripts/run_macos_gui.sh`

## Quick checklist (do this first next time)

- XQuartz is running; Security allows network clients; run `xhost + 127.0.0.1`.
- Use the launcher: `scripts/run_macos_gui.sh` (it sets a good `DISPLAY`).
- Apple Silicon? If build fails: `export DOCKER_DEFAULT_PLATFORM=linux/amd64` and retry.
- Verify compose validity: `docker compose -f docker/docker-compose.yml config`.

## If it still fails

- Inside the container: `echo $DISPLAY` (expect `host.docker.internal:0`).
- Optional test: `apt-get update && apt-get install -y x11-apps && xeyes` (should open a window).
- Check ROS/panda logs printed by `docker compose up`.
- Try toggling llvmpipe: remove `MESA_LOADER_DRIVER_OVERRIDE` to see if hardware GL works.

## Files touched for the fix

- `scripts/run_macos_gui.sh` — forces TCP DISPLAY if needed; validates compose.
- `docker/docker-compose.yml` — clean env mapping; GL fallbacks; no `/tmp/.X11-unix` on macOS.
- `docker/Dockerfile` — installs `panda3d-gltf`.
- `docker/ros_entrypoint.sh` — GUI-friendly backends and Panda3D hints.

## One-liner launcher (recommended)

```bash
scripts/run_macos_gui.sh
```

This is the known-good path for macOS GUI in Docker with Panda3D.
