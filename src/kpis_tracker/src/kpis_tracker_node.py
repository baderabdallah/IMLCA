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
        out_topic = "/kpi"

        # Data used to track speed KPIs        
        self.overall_sum_of_velocity_samples = 0
        self.overall_count_of_velocity_samples = 0
        self.overall_avg_of_velocity_samples = 0
        self.current_lane = None
        self.lane_changes = 0
        self.kpi = KPIs()

        # Publisher
        self.pub = rospy.Publisher(out_topic, KPIs, queue_size=10)

        # Subscriber
        self.sub = rospy.Subscriber(in_topic, Mlc, self._subscriber_callback)

        # Set the rate for publishing
        self.publish_rate = rospy.Rate(rate)
   
    def _subscriber_callback(self, msg):
        self.latest_msg = msg
        self._update_overall_avg_velocity(msg)
        self._update_lane_changes(msg)

    def _update_overall_avg_velocity(self, msg):
        # Compute the average speed just accumulating the
        # current speed and dividing by number of samples
        # elapsed since now
        # TODO: could have numerical instability in long run
        self.overall_count_of_velocity_samples += 1
        self.overall_sum_of_velocity_samples += msg.ego_info.velocity_x
        self.overall_avg_of_velocity_samples = self.overall_sum_of_velocity_samples / self.overall_count_of_velocity_samples

    def _update_lane_changes(self, msg):
        # (possibly) update the count for lane changes
        if self.current_lane is not None and self.current_lane != msg.ego_info.lane_number:
            self.lane_changes += 1
        self.current_lane = msg.ego_info.lane_number

    def _publishing(self):
        # Publish the KPI at specified rate
        while not rospy.is_shutdown():
            self.kpi.lane_changes = self.lane_changes
            self.kpi.overall_avg_velocity = self.overall_avg_of_velocity_samples
            self.pub.publish(self.kpi)
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
