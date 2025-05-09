# Multi Lane Change Assist
## 3D Simulation
Link to simulation video:

https://dxcportal.sharepoint.com/:v:/s/ADFCprojectIntellignetmultilaneassist/EZz_U3apEfxBjqG6x12P6TcBeoNHoOj3PYLOitpiYkPjHw?e=XIvomM&nav=eyJyZWZlcnJhbEluZm8iOnsicmVmZXJyYWxBcHAiOiJTdHJlYW1XZWJBcHAiLCJyZWZlcnJhbFZpZXciOiJTaGFyZURpYWxvZy1MaW5rIiwicmVmZXJyYWxBcHBQbGF0Zm9ybSI6IldlYiIsInJlZmVycmFsTW9kZSI6InZpZXcifX0%3D

## Requirements
- Ubuntu 20.04
- ROS Noetic
- The 'Panda3D' framework (https://www.panda3d.org)
- The 'panda3d-gltf' plugin

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

## Building and Running
From the workspace folder run
```console
$ catkin_make
```
source the workspace
```console
$ source devel/setup.sh
```
launch the MLC assist
```console
$ roslaunch multi_lane_assist.launch
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

