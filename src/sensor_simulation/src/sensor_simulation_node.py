#!/usr/bin/env python

import rospy
from lane_msgs.msg import Mlc, ObjectList, Object

class SensorSimulationNode():
    """
    ROS node implementing class for receiving and plotting the published MLC data.
    """

    def __init__(self):
        mlc_topic = "/mlc/mlc_data"
        self.object_list = []
        
        self.mlc_subscriber = rospy.Subscriber(
            mlc_topic, Mlc, self.mlc_callback
        )

        topic_name = "sensor_simulation/object_list"
        self.scenario_publisher = rospy.Publisher(topic_name, ObjectList, queue_size=10)


    def calculate_y(self, lane_id, number_of_lanes):
        lane_width = 4
        return (number_of_lanes - lane_id) * lane_width - (lane_width/2)

    def mlc_callback(self, msg):
        ego_x = msg.ego_vehicle_trajectory.trajectory[0].x
        ego_y = msg.ego_vehicle_trajectory.trajectory[0].y
        ego_vx = msg.ego_info.velocity_x
        ego_lane_number = msg.ego_info.lane_number

        ego_info = [
            {
                "x": ego_x,
                "y": ego_y,
                "vx": ego_vx,
                "vy": 0,
            }
        ]

        lanes_number = msg.scenario_data.number_of_lanes
        self.object_list = ego_info + [{
            "x": data.pos_x,
            "y": self.calculate_y(data.lane_number, lanes_number),
            "vx": data.velocity_x,
            "vy": 0,
        } for data in msg.scenario_data.vehicles_information]

        self.publish_objects()

    def convert_to_topic(self, input):
        object_list_ros = ObjectList()

        object_list_ros.object_list =  [
            self._convert_object(object) for object in input
        ]

        return object_list_ros

    def _convert_object(self, object):
        object_ros = Object()
        object_ros.x =  object["x"]
        object_ros.y =  object["y"]
        object_ros.vx = object["vx"]
        object_ros.vy = object["vy"]
        return object_ros

    def publish_objects(self):
        topic = self.convert_to_topic(self.object_list)
        self.scenario_publisher.publish(topic)

    def spin(self):
        rospy.spin()


# Main function.
if __name__ == '__main__':

    rospy.init_node('sensor_simulation_node')
    try:
        sensor_simulation_node = SensorSimulationNode()
        sensor_simulation_node.spin()

    except rospy.ROSInterruptException as err:
        print("[Sensor Simulation] An error occurred:", err)
