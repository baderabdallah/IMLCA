#!/usr/bin/env python

import rospy
from lane_msgs.msg import Mlc

class KPIsTrackerNode():
    """
    ROS node implementing class for monitoring data and computing KPI information
    """

    def __init__(self):
        mlc_topic = "/mlc/mlc_data"
        
        self.mlc_subscriber = rospy.Subscriber(
            mlc_topic, Mlc, self.mlc_callback)

        self.overall_sum_of_velocity_samples = 0
        self.overall_count_of_velocity_samples = 0
        self.overall_avg_of_velocity_samples = 0

    def mlc_callback(self, msg):
        self.latest_msg = msg
        self.update_overall_avg_velocity(msg)

    def spin(self):
        rospy.spin()

    def update_overall_avg_velocity(self, msg):
        self.overall_count_of_velocity_samples += 1
        self.overall_sum_of_velocity_samples += msg.ego_info.velocity_x
        self.overall_avg_of_velocity_samples = self.overall_sum_of_velocity_samples / self.overall_count_of_velocity_samples

# Main function.
if __name__ == '__main__':

    rospy.init_node('kpis_tracker_node')
    try:
        kpis_tracker_node = KPIsTrackerNode()
        kpis_tracker_node.spin()

        raise RuntimeError()

    except rospy.ROSInterruptException as err:
        print(f"[KPIs tracker] An error occured: {err}")
