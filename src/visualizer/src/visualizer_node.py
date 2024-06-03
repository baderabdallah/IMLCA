#!/usr/bin/env python

from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt
import rospy
from lane_msgs.msg import Mlc
from plotter.scenario_plotter import plot_mlc
from plotter.constants import *


class VisualizerNode():
    """
    ROS node implementing class for receving and plotting the published MLC data.
    """

    def __init__(self):
        mlc_topic = "/mlc/mlc_data"
        self._fig = plt.figure(
            num=FIGURE_TITLE, figsize=(FIGURE_WIDTH, FIGURE_HEIGHT))

        self._ax = self._fig.add_axes([0.04, 0.075, 0.94, 0.85], facecolor=STREET_COLOR)

        self.mlc_subscriber = rospy.Subscriber(
            mlc_topic, Mlc, self.mlc_callback)

        self.ani = FuncAnimation(self._fig, self.update_plot, cache_frame_data=False)


    def mlc_callback(self, msg):
        self.latest_msg = msg

    def update_plot(self, _):
        if hasattr(self, 'latest_msg'):
            plot_mlc(self.latest_msg, self._ax)

    def spin(self):
        plt.show()
        rospy.spin()


# Main function.
if __name__ == '__main__':

    rospy.init_node('visualizer_node')
    try:
        visualizer_node = VisualizerNode()
        visualizer_node.spin()

    except rospy.ROSInterruptException as err:
        print("[Visualization] An error occured:", err)
