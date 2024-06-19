"""
@file scenario_plottyer.py
@copyright Copyright (C) 2021 BMW Hackaton Team1
@brief This file contains functions to plot the content of an MLC message on a given axis.
"""

import rospy
from plotter.constants import *
from plotter.objects.car import Car
from plotter.objects.trajectory import Trajectory
from plotter.objects.scene2D import Scene2D
from plotter.helpers import *

def get_min_max_x(mlc_message):
    """
    Retrieves the first and last X coordinate of the road section which has be visualized.
    The aim is to retrieve [ego_x - LOOK_BEHIND, last_ego_trajectory_x + LOOK_AHEAD].
    The information
        @param mlc_message: the received MLC message
        @return: the minimum and maximum relevant X coordinates in the current scenario.
    """

    min_x = -LOOK_BEHIND
    max_x = LOOK_AHEAD

    if mlc_message.ego_vehicle_trajectory.trajectory:
        # Ego pos
        ego_x = mlc_message.ego_vehicle_trajectory.trajectory[0].x
        min_x = ego_x - LOOK_BEHIND
        max_x = ego_x + LOOK_AHEAD

    elif mlc_message.scenario_data.vehicles_information:
        xs = []
        for vehicle in mlc_message.scenario_data.vehicles_information:
            xs.append(vehicle.pos_x)
        min_vehicles_x = min(xs)
        max_vehicles_x = max(xs)

        min_x = min_vehicles_x - LOOK_BEHIND
        max_x = max_vehicles_x + LOOK_BEHIND

    return (min_x, max_x)

def __make_title(scenario):
    timestamp = scenario.header.stamp
    return f"Frame #{scenario.header.seq}, time: {timestamp.secs}.{timestamp.nsecs}"

def __get_trajectory(mlc_message):
    trajectory = mlc_message.ego_vehicle_trajectory.trajectory
    
    if not trajectory:
        header = mlc_message.scenario_data.header
        timestamp = header.stamp
        print(
            f"* Message #{header.seq} contains an empty trajectory (at {timestamp.secs}.{timestamp.nsecs} seconds)")
        return []
    
    return trajectory
    
def build_ego_from_trajectory_data(trajectory):
    ego_x = trajectory[0].x
    ego_y = trajectory[0].y
    
    heading = calculate_angle(trajectory)
    print("heading:", heading)
    
    return Car(
        x=ego_x,
        y=ego_y,
        length=CAR_LENGTH,
        width=CAR_WIDTH,
        heading_angle_deg=heading,
        is_ego=True,
    )
    
def build_traffic_agents(vehicles_information, lane_numbers, scene):
     # Collect car position data
    xs = [vehicle.pos_x for vehicle in vehicles_information]
    ys = [get_y_coordinate_from_lane_number(lane_numbers, vehicle.lane_number) 
            for vehicle in vehicles_information]
            
    # Create cars objects
    return [
        Car(
            x=x,
            y=y,
            length=CAR_LENGTH,
            width=CAR_WIDTH,
            heading_angle_deg=0,
            is_ego=False,
        )
        for x, y in zip(xs, ys)
        ]
    

def plot_mlc(mlc_message, ax):

    rospy.loginfo(f"Number of lanes: {mlc_message.scenario_data.number_of_lanes}")

    (min_x, max_x) = get_min_max_x(mlc_message)
    
    scene = Scene2D(
        ax=ax,
        min_x=min_x,
        max_x=max_x,
        number_of_lanes=mlc_message.scenario_data.number_of_lanes,
        title=__make_title(mlc_message.scenario_data),
    )
    
    trajectory_data = __get_trajectory(mlc_message=mlc_message)    
    
    if len(trajectory_data):
        
        # Add trajectory to the scene
        trajectory = Trajectory(
            trajectory_data
        )
        scene.add(trajectory)
    
        # Add ego vehicle to the scene
        ego = build_ego_from_trajectory_data(trajectory_data)
        scene.add(ego)
    
    # Add other vehicles to the scene
    vehicles_list = build_traffic_agents(
            mlc_message.scenario_data.vehicles_information, 
            mlc_message.scenario_data.number_of_lanes,
            scene,
        )
    
    #for vehicle in vehicles_list:
    scene.add(*vehicles_list)
    
    # Draw the scene
    scene.plot()