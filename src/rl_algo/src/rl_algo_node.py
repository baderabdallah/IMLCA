#!/usr/bin/env python

import rospy
import std_msgs.msg
from lane_msgs.msg import ObjectList
from std_msgs.msg import Float32MultiArray

import gymnasium as gym
from gymnasium.wrappers import RecordVideo
from stable_baselines3 import DQN

import numpy as np
import os, rospkg

from rl_agents.agents.common.factory import agent_factory


def convert_action_to_string(action):

    rospy.loginfo(f"ACTION {type(action)}")

    map_action = {
        0: "Up Arrow",
        1: "Idle",
        2: "Down Arrow",
        3: "Right Arrow",
        4: "Left Arrow",
    }

    return map_action[int(action)]


class RL:
    def __init__(self) -> None:
        rospack = rospkg.RosPack()
        node_path = rospack.get_path("rl_algo")

        env = gym.make("highway-v0", render_mode="rgb_array")

        # Create the model
        self.model = DQN(
            "MlpPolicy",
            env,
            policy_kwargs=dict(net_arch=[256, 256]),
            learning_rate=5e-4,
            buffer_size=15000,
            learning_starts=200,
            batch_size=32,
            gamma=0.8,
            train_freq=1,
            gradient_steps=1,
            target_update_interval=50,
            verbose=1,
            tensorboard_log="models/model",
        )
        # Run the trained model and record video
        self.model = DQN.load("/home/abader/imlca/src/rl_algo/src/models/model", env=env)

    def get_action(self, observation):
        action, _states = self.model.predict(observation, deterministic=True)
        return action


class RLAlgoNode:
    def __init__(self, rl: RL):

        self.rl = rl
        self.object_list = []
        # self.object_list_subscriber = rospy.Subscriber(
        #     "sensor_simulation/object_list", ObjectList, self.object_list_callback
        # )
        self.observation_subscriber = rospy.Subscriber(
            "/mlc/transformed_state", Float32MultiArray, self.observation_callback
        )

        topic_name = "keyboard_input"
        self.action_publisher = rospy.Publisher(topic_name, std_msgs.msg.String, queue_size=1)

    def observation_callback(self, msg):
        # Extract the array data and transform it to a NumPy array
        array_data = np.array(msg.data, dtype=np.float32)

        # Reshape the NumPy array to 5x5 based on the known dimensions
        observation = array_data.reshape((5, 5))

        # Log the received array and its NumPy transformation
        rospy.loginfo(observation)
        action = self.rl.get_action(observation)
        rospy.loginfo(action)
        if action != int(1):
            msg = convert_action_to_string(action)
            self.action_publisher.publish(msg)

    def object_list_callback(self, msg):

        self.object_list = [self.get_object_to_dict(object) for object in msg.object_list]

        if len(self.object_list) > 5:
            self.object_list = self.object_list[:5]

        dict_of_list =  np.array([[dic[k] for dic in self.object_list] for k in self.object_list[0]])

        for i in range(len(dict_of_list)):
            dict_of_list[i] = dict_of_list[i] / max(dict_of_list[i])

        dict_of_list = dict_of_list.T

        rospy.loginfo(f"TEST: {dict_of_list}")
        action = self.rl.get_action(dict_of_list)
        
        msg = convert_action_to_string(action)
        self.action_publisher.publish(msg)

    def get_object_to_dict(self, object):
        return {
            "presence": 1,
            "x": object.x,
            "y": object.y,
            "vx": object.vx,
            "vy": object.vy,
        }



# Main function.
if __name__ == '__main__':
    rospy.init_node('scenario_generator')
    try:

        rl = RL()
        scenario_node = RLAlgoNode(rl)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
