#!/usr/bin/env python

import unittest
import rospy
from lane_msgs.msg import ScenarioData
#from time import sleep
import rostest

class ScenarioGeneratorTestCase(unittest.TestCase):

	is_scenario_data_received = False

	def callback(self, data):
		self.is_scenario_data_received = True

	def test_if_scenario_data_published(self):
		rospy.init_node('test_scenario_generator')
		rospy.Subscribe('mlc/scenario_information', ScenarioData, self.callback)

		counter = 0
		while (not rospy.is_shutdown()) and (counter < 5) and (not self.is_scenario_data_received):
			print("Sleeping {}...".format(counter))
			#sleep (1)
			counter += 1

		self.assertTrue(self.is_scenario_data_received)


if __name__ == '__main__':
	rostest.rosrun('scenario_builder', 'test_scenario_generator', ScenarioGeneratorTestCase)
