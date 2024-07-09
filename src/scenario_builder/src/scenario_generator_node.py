#!/usr/bin/env python

import rospy
import std_msgs.msg
from lane_msgs.msg import ScenarioData
from lane_msgs.msg import VehiclePosition

class ScenarioGenerator:
    def __init__(self, number_of_lanes, vehicles_initial_info):
        self.number_of_lanes = number_of_lanes
        self.vehicle_info = vehicles_initial_info
        self.counter = 0

    def get_vehicles_position(self):
        vehicle_current_positions = []
        for id, vehicle in enumerate(self.vehicle_info):
            vehicle_position = VehiclePosition()
            vehicle_position.lane_number = vehicle["lane_number"]
            vehicle_position.pos_x = (self.counter * vehicle["speed"] * 0.1) + vehicle["initial_pos_x"]
            vehicle_position.id  = id
            vehicle_position.velocity_x = vehicle["speed"]

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

class ScenarioNode:
    def __init__(self):
        self.rate = rospy.Rate(5) # 10hz
        topic_name = "mlc/scenario_information"
        self.scenario_publisher = rospy.Publisher(topic_name, ScenarioData, queue_size=10)


    def publish_scenario(self, scenario_generator):
        while not rospy.is_shutdown():
            msg = scenario_generator.get_scenario_data()
            self.scenario_publisher.publish(msg)
            self.rate.sleep()


def get_vehicles_initial_info():
    vehicles_initial_info = [
        {"lane_number": 2, "speed": 13.0, "initial_pos_x": 70.0},
        {"lane_number": 3, "speed": 13.0, "initial_pos_x": 60.0},
        {"lane_number": 2, "speed": 13.0, "initial_pos_x": 190.0},
        {"lane_number": 1, "speed": 14.5, "initial_pos_x": 80.0},
        {"lane_number": 1, "speed": 13.0, "initial_pos_x": 150.0},
        {"lane_number": 0, "speed": 14.5, "initial_pos_x": 100.0},
        {"lane_number": 3, "speed": 13.5, "initial_pos_x": 250.0},
        {"lane_number": 0, "speed": 13.5, "initial_pos_x": 220.0},
        {"lane_number": 2, "speed": 13.5, "initial_pos_x": 260.0},
        {"lane_number": 2, "speed": 13.0, "initial_pos_x": 125.0},
    ]

    return vehicles_initial_info


# Main function.
if __name__ == '__main__':
    rospy.init_node('scenario_generator')
    try:

        scenario_node = ScenarioNode()

        number_of_lanes = 4
        vehicles_initial_info =  get_vehicles_initial_info()
        ego_speed = 16.66 # TODO: is this really used? See 90kph in mlc files.
        scenario_generator = ScenarioGenerator(number_of_lanes, vehicles_initial_info)

        scenario_node.publish_scenario(scenario_generator)
    except rospy.ROSInterruptException:
        pass
