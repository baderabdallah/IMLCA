import unittest
import roslaunch
import rospy
import subprocess
import os

from .kpis_tracker_node import KPIsTrackerNode
from lane_msgs.msg import Mlc
from lane_msgs.msg import KPIs


class TestKPIs(unittest.TestCase):
    def test_avg_velocity(self):
        # Test if the average velocity is computed correctly
        # providing a speed ramp from 1 to N and check
        # if the avarage is equal to (N + 1)/2 that is
        # the average of first N number

        # Initialize the node in order to receive the KPIs topic
        rospy.init_node("KPIs_subscriber", anonymous=True)
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        package_path = os.path.dirname(os.path.dirname(__file__))
        launch_file_path = os.path.join(package_path, "test", "test_KPIs.launch")
        launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_file_path])
        launch.start()

        # Create the KPIs node
        test_node = KPIsTrackerNode()

        # This is the topic listen by KPIs node
        mock_pub = rospy.Publisher("mlc/mlc_data", Mlc, queue_size=10)
        # Give the publisher time to connect
        rospy.sleep(2)

        test_message = Mlc()
        excepted_avg = 0
        N = 15
        # We are pretending that velocity is going from 1 to N (15)
        # increasing of 1 at each step, so that means that
        # the average at each step is given by: (1 + ... + i) / i
        # that is the well known Gauss series
        # and its value is given by (i + 1) / 2
        for v in range(1, N):
            # Update speed
            test_message.ego_info.velocity_x = v
            mock_pub.publish(test_message)
            excepted_avg = (v + 1) / 2

            received_message = rospy.wait_for_message("kpi", KPIs, timeout=5)
            assert received_message.overall_avg_velocity == excepted_avg

        # Shutdown the launch and roscore
        launch.shutdown()

    def test_lane_changes(self):
        # Test if the number of lane changes is tracked correctly

        # Initialize the node in order to receive the KPIs topic
        rospy.init_node("KPIs_subscriber", anonymous=True)
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        package_path = os.path.dirname(os.path.dirname(__file__))
        launch_file_path = os.path.join(package_path, "test", "test_KPIs.launch")
        launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_file_path])
        launch.start()

        # Create the KPIs node
        test_node = KPIsTrackerNode()

        # This is the topic listen by KPIs node
        mock_pub = rospy.Publisher("mlc/mlc_data", Mlc, queue_size=10)
        # Give the publisher time to connect
        rospy.sleep(2)

        test_message = Mlc()
        N = 15

        for lane in range(1, N):
            # Update change lanes
            test_message.ego_info.lane_number = lane % 4
            mock_pub.publish(test_message)
            received_message = rospy.wait_for_message("kpi", KPIs, timeout=5)
            assert received_message.lane_changes == lane - 1

        # Shutdown the launch and roscore
        launch.shutdown()
