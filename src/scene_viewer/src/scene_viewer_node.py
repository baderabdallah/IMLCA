#!/usr/bin/env python

import rospy
from lane_msgs.msg import Mlc
from lane_msgs.msg import KPIs
from scene_viewer.constants import *
from scene_viewer.scene_viewer import SceneViewer


class SceneViewerNode:
    """
    ROS node implementing class for receving and plotting the published MLC data.
    """

    def __init__(self):
        mlc_topic = "/mlc/mlc_data"
        kpi_topic = "/kpi"

        self.scene_viewer = SceneViewer()

        self.mlc_subscriber = rospy.Subscriber(mlc_topic, Mlc, self.mlc_callback)
        self.kpi_subscriber = rospy.Subscriber(kpi_topic, KPIs, self.kpi_callback)

    def mlc_callback(self, msg):
        self.latest_msg = msg
        self.update_plot(frame=None)

    def kpi_callback(self, msg):
        self.kpi_msg = msg
        self.update_plot(frame=None)

        
    def update_plot(self, frame):
        if hasattr(self, "latest_msg") and hasattr(self, "kpi_msg"):
            self.scene_viewer.update(self.latest_msg, self.kpi_msg)

    def spin(self):
        self.scene_viewer.run()
        rospy.spin()


# Main function.
if __name__ == "__main__":

    rospy.init_node("scene_viewer_node")
    try:
        scene_viewer_node = SceneViewerNode()
        scene_viewer_node.spin()

    except rospy.ROSInterruptException as err:
        print("[Scene Viewer] An error occured:", err)
