#!/usr/bin/env python

import rospy
import std_msgs.msg
from lane_msgs.msg import ScenarioData
from lane_msgs.msg import VehicleInfo


class ScenarioGenerator:
    def __init__(self, number_of_lanes, vehicles_initial_info):
        self.number_of_lanes = number_of_lanes
        self.vehicle_info = vehicles_initial_info
        self.counter = 0

    def get_vehicles_position(self):
        vehicle_current_positions = []
        for id, vehicle in enumerate(self.vehicle_info):
            vehicle_position = VehicleInfo()
            vehicle_position.lane_number = vehicle["lane_number"]
            vehicle_position.pos_x = (self.counter * vehicle["speed"] * 0.1) + vehicle[
                "initial_pos_x"
            ]
            vehicle_position.id = id

            vehicle_current_positions.append(vehicle_position)

        self.counter += 1
        return vehicle_current_positions

    def get_scenario_data(self):
        scenario_data = ScenarioData()
        scenario_data.header = std_msgs.msg.Header()
        scenario_data.header.stamp = rospy.Time.now()
        scenario_data.header.seq = self.counter
        scenario_data.number_of_lanes = self.number_of_lanes
        scenario_data.vehicles_information = self.get_vehicles_position()

        return scenario_data
