#!/usr/bin/env python

import rospy
import threading
from lane_msgs.msg import Mlc
from lane_msgs.msg import KPIs

class KPIsTrackerNode():
    """
    ROS node implementing class for monitoring data and computing KPI information
    """

    def __init__(self, rate = 2):
        in_topic = "/mlc/mlc_data"
        out_topic = "/kpi/data"
        
        self.overall_sum_of_velocity_samples = 0
        self.overall_count_of_velocity_samples = 0
        self.overall_avg_of_velocity_samples = 0

        # Publisher
        self.pub = rospy.Publisher(out_topic, KPIs, queue_size=10)

        # Subscriber
        self.sub = rospy.Subscriber(in_topic, Mlc, self._subscriber_callback)

        # Set the rate for publishing
        self.publish_rate = rospy.Rate(rate)
   
    def _subscriber_callback(self, msg):
        self.latest_msg = msg
        self._update_overall_avg_velocity(msg)

    def _update_overall_avg_velocity(self, msg):
        self.overall_count_of_velocity_samples += 1
        self.overall_sum_of_velocity_samples += msg.ego_info.velocity_x
        self.overall_avg_of_velocity_samples = self.overall_sum_of_velocity_samples / self.overall_count_of_velocity_samples

    def _publishing(self):
        while not rospy.is_shutdown():
            kpi = KPIs()
            kpi.lane_changes = 0
            kpi.overall_avg_velocity = self.overall_avg_of_velocity_samples
            self.pub.publish(kpi)
            self.publish_rate.sleep()

    def start(self):
        # Start the publishing thread
        pub_thread = threading.Thread(target=self._publishing)
        pub_thread.start()

        # Keep the main thread alive
        rospy.spin()

# Main function
if __name__ == '__main__':

    rospy.init_node('kpis_tracker_node')
    try:
        kpis_tracker_node = KPIsTrackerNode()
        kpis_tracker_node.start()

    except rospy.ROSInterruptException as err:
        print(f"[KPIs tracker] An error occured: {err}")
