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

    def mlc_callback(self, msg):
        self.latest_msg = msg

    def spin(self):
        rospy.spin()        

# Main function.
if __name__ == '__main__':

    rospy.init_node('kpis_tracker_node')
    try:
        kpis_tracker_node = KPIsTrackerNode()
        kpis_tracker_node.spin()

    except rospy.ROSInterruptException as err:
        print(f"[KPIs tracker] An error occured: {err}")
