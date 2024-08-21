#!/usr/bin/env python

from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt
import rospy
from lane_msgs.msg import Mlc
from include.constants import *
from include.speed_visual import SpeedVisual


class SpeedVisualizerNode:
    """
    ROS node implementing class for receiving and plotting the published MLC data.
    """

    def __init__(self):
        mlc_topic = "/mlc/mlc_data"

        self.speed_visual = SpeedVisual()

        self.mlc_subscriber = rospy.Subscriber(mlc_topic, Mlc, self.mlc_callback)

        self.anim = FuncAnimation(
            self.speed_visual._fig, self.update_plot, cache_frame_data=False
        )

    def mlc_callback(self, msg):
        self.latest_msg = msg

    def update_plot(self, _):
        if hasattr(self, "latest_msg"):
            self.speed_visual.draw_speedometer_visual(
                self.latest_msg.ego_info.velocity_x
            )

    def spin(self):
        plt.show()
        rospy.spin()


# Main function.
if __name__ == "__main__":

    rospy.init_node("speed_visualizer_node")
    try:
        visualizer_node = SpeedVisualizerNode()
        visualizer_node.spin()

    except rospy.ROSInterruptException as err:
        print("[Speed Visual] An error occurred:", err)
