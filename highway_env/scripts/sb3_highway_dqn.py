import gymnasium as gym
from gymnasium.wrappers import RecordVideo
from stable_baselines3 import DQN

import highway_env  # noqa: F401

TRAIN = False

if __name__ == "__main__":
    # Create the environment
    env = gym.make("highway-fast-v0", render_mode="rgb_array")
    # Configure the base env (not the Gymnasium wrapper)
    env.unwrapped.configure(
        {
            "observation": {"type": "Kinematics"},
            "action": {
                "type": "DiscreteMetaAction",
            },
            "lanes_count": 4,
            "vehicles_count": 15,
            "duration": 40,  # [s]
            "initial_spacing": 2,
            "collision_reward": -1,  # The reward received when colliding with a vehicle.
            "reward_speed_range": [
                20,
                30,
            ],  # [m/s] The reward for high speed is mapped linearly from this range to [0, HighwayEnv.HIGH_SPEED_REWARD].
            "simulation_frequency": 15,  # [Hz]
            "policy_frequency": 1,  # [Hz]
            "other_vehicles_type": "highway_env.vehicle.behavior.IDMVehicle",
            "screen_width": 600,  # [px]
            "screen_height": 150,  # [px]
            "centering_position": [0.3, 0.5],
            "scaling": 5.5,
            "lane_change_reward": 0.3,  # The reward received at each lane change action.
            "right_lane_reward": 0.05,  # The reward received when driving on the right-most lanes, linearly mapped to
            "high_speed_reward": 0.8,  # The reward received when driving at full speed, linearly mapped to zero for
            "show_trajectories": True,
            "render_agent": True,
        }
    )

    obs, info = env.reset()

    # Create the model
    model = DQN(
        "MlpPolicy",
        env,
        policy_kwargs=dict(net_arch=[256, 256]),
        learning_rate=0.001,
        buffer_size=35000,
        learning_starts=1000,
        batch_size=32,
        gamma=0.8,
        train_freq=1,
        gradient_steps=1,
        target_update_interval=50,
        verbose=1,
        tensorboard_log="highway_dqn/",
        exploration_final_eps=0.001,
        seed=99,
    )

    # Train the model
    if TRAIN:
        model.learn(total_timesteps=int(2e4))
        model.save("highway_dqn/model")
        del model

    # Run the trained model and record video
    model = DQN.load("highway_dqn/model", env=env)
    # Optional: adjust FPS for rendering on base env before wrapping
    env.unwrapped.configure({"simulation_frequency": 15})
    # Wrap with Gymnasium's RecordVideo
    env = RecordVideo(
        env,
        video_folder="highway_dqn/videos",
        episode_trigger=lambda e: True,
    )

    for videos in range(10):
        done = truncated = False
        obs, info = env.reset()
        while not (done or truncated):
            # Predict
            action, _states = model.predict(obs, deterministic=True)
            # Get reward
            obs, reward, done, truncated, info = env.step(action)
            # Render
            env.render()
    env.close()
