# Multi Lane Change Assist
 
## How to run (macOS GUI)

From the repo root, run:

```bash
scripts/run_macos_gui.sh
```

## 3D Simulation
Link to simulation video:

[Intelligent Highway Pilot 3D Demo](https://dxcportal.sharepoint.com/:v:/r/sites/ADFCprojectIntellignetmultilaneassist/Shared%20Documents/General/Intelligent%20Highway%20Pilot%203D%20Demo.mp4?csf=1&web=1&e=AF0Zed&nav=eyJyZWZlcnJhbEluZm8iOnsicmVmZXJyYWxBcHAiOiJTdHJlYW1XZWJBcHAiLCJyZWZlcnJhbFZpZXciOiJTaGFyZURpYWxvZy1MaW5rIiwicmVmZXJyYWxBcHBQbGF0Zm9ybSI6IldlYiIsInJlZmVycmFsTW9kZSI6

## Requirements
Native (Linux/ROS):
- Ubuntu 20.04
- ROS Noetic
- The 'Panda3D' framework (https://www.panda3d.org)
- The 'panda3d-gltf' plugin

Docker (macOS/Windows recommended):
- Docker Desktop
- For macOS GUI: XQuartz (https://www.xquartz.org)

## Description

TODO: Update description

## Installing required packages
Install the 'Panda3D' framework, version 1.10.14 or above
```console
$ pip install panda3d==1.10.14
```

Install the 'panda3d-gltf' plugin
```console
$ pip install -U panda3d-gltf
```

## Run on macOS with GUI (Docker + XQuartz)
1) Install and start XQuartz. In Preferences > Security, check "Allow connections from network clients". Restart XQuartz, then in an XQuartz terminal run: `xhost + 127.0.0.1`
2) From the repo root:
    - Default GUI launch (2D + Panda3D viewers):
       - `scripts/run_macos_gui.sh`
    - Alternate visuals:
       - `LAUNCH_FILE=manual_controller_visuals.launch scripts/run_macos_gui.sh`
       - `LAUNCH_FILE=manual_controller_panda3d_visuals.launch scripts/run_macos_gui.sh`

Notes:
- On Apple Silicon, you may need to set `DOCKER_DEFAULT_PLATFORM=linux/amd64` before first run.
- If the GUI window doesn't appear, verify XQuartz is running and `echo $DISPLAY` prints something like `host.docker.internal:0`.

Troubleshooting playbook: see `docs/AI_README_macOS_GUI.md`.

## Building and Running (native Ubuntu/ROS)
From the workspace folder run
```console
$ catkin_make
```
source the workspace
```console
$ source devel/setup.sh
```
launch the MLC assist (choose one of the launch files):
```console
$ roslaunch manual_controller_visuals.launch
```

## ROS node structure
- scenario_generator_node
    - Publications:
        - /mlc/scenario_information [lane_msgs/ScenarioData]

 - mlc_node
    - Publications:
       - /mlc/mlc_data [lane_msgs/Mlc]

    - Subscriptions:
        - /mlc/scenario_information [lane_msgs/ScenarioData]

  - kpi_node
    - Publications:
       - /kpi [lane_msgs/KPIs]

    - Subscriptions:
        - /mlc/mlc_data [lane_msgs/Mlc]

TODO: add description of scene viewer node

