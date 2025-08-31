#!/usr/bin/env python

import rospy
import std_msgs.msg
from std_msgs.msg import Float32MultiArray

import gymnasium as gym
from gymnasium.wrappers import RecordVideo
import sys
import os, rospkg
import zipfile, tempfile, shutil, io
import numpy as np

# Early, aggressive NumPy compatibility shims for NumPy<2 loading pickles saved with NumPy>=2
try:
    import importlib, types
    import numpy as _np
    _np_major = int(str(_np.__version__).split('.')[0])
    if _np_major < 2:
        # Alias numpy._core -> numpy.core and common submodules
        try:
            import numpy.core as _np_core  # type: ignore
            sys.modules.setdefault('numpy._core', _np_core)
            _aliases = {
                'numpy._core._multiarray_umath': 'numpy.core._multiarray_umath',
                'numpy._core.multiarray': 'numpy.core.multiarray',
                'numpy._core.numerictypes': 'numpy.core.numerictypes',
                'numpy._core.overrides': 'numpy.core.overrides',
                'numpy._core.fromnumeric': 'numpy.core.fromnumeric',
                'numpy._core.shape_base': 'numpy.core.shape_base',
            }
            for alias, target in _aliases.items():
                try:
                    sys.modules.setdefault(alias, importlib.import_module(target))
                except Exception:
                    pass
        except Exception:
            pass

        # Ensure a module exists at numpy.random._pcg64 exposing PCG64
        try:
            import numpy.random as _npr
            _pcg64_mod = types.ModuleType('numpy.random._pcg64')
            if hasattr(_npr, 'PCG64'):
                _pcg64_mod.PCG64 = _npr.PCG64  # type: ignore[attr-defined]
            else:
                class _DummyBitGen(object):
                    def __init__(self, seed=None):
                        self._bitgen = _npr.MT19937(seed)
                _pcg64_mod.PCG64 = _DummyBitGen  # type: ignore[attr-defined]
            sys.modules['numpy.random._pcg64'] = _pcg64_mod
        except Exception:
            pass

        # Patch numpy.random._pickle to always resolve any BitGenerator name to PCG64 as a last resort
        try:
            import numpy.random as _npr
            import numpy.random._pickle as _nrp  # type: ignore
            # Populate BitGenerators with likely keys
            try:
                _PCG = getattr(_npr, 'PCG64', None)
                if _PCG is not None:
                    for k in (
                        str(_PCG),
                        "<class 'numpy.random._pcg64.PCG64'>",
                        "<class 'numpy.random.bit_generator.PCG64'>",
                        "<class 'numpy.random._bit_generator.PCG64'>",
                        'numpy.random._pcg64.PCG64',
                        'numpy.random._bit_generator.PCG64',
                    ):
                        _nrp.BitGenerators[k] = _PCG
            except Exception:
                pass

            _orig_ctor = getattr(_nrp, '__bit_generator_ctor', None)
            def _force_pcg64_ctor(name):  # type: ignore
                try:
                    # Always prefer returning a valid BitGenerator class; PCG64 exists in NumPy>=1.17
                    if hasattr(_npr, 'PCG64'):
                        return _npr.PCG64
                    # Fallback to MT19937 to avoid crashing
                    return _npr.MT19937
                except Exception:
                    # If anything goes wrong, try original
                    if _orig_ctor is not None:
                        return _orig_ctor(name)  # type: ignore[misc]
                    raise
            # Overwrite the constructor used during unpickling
            _nrp.__bit_generator_ctor = _force_pcg64_ctor  # type: ignore
            rospy.loginfo('[rl_algo] Applied aggressive NumPy BitGenerator shim (PCG64 force)')
        except Exception:
            pass

        # Wrap cloudpickle.loads to rewrite _pcg64 path inside the bytestream
        try:
            import cloudpickle as _cp
            _orig_cp_loads = _cp.loads
            def _cp_loads_shim(b, *args, **kwargs):  # type: ignore
                # First try original loads
                try:
                    return _orig_cp_loads(b, *args, **kwargs)
                except Exception as e:
                    # If it fails due to PCG64 issue, try byte-level replacement
                    if isinstance(b, (bytes, bytearray)) and b'numpy.random._pcg64' in b:
                        rospy.loginfo('[rl_algo] Attempting NumPy 2.x -> 1.x byte-level patch during unpickle')
                        try:
                            # Replace _pcg64 with correct numpy 1.x module path
                            b_new = b.replace(b"numpy.random._pcg64.PCG64", b"numpy.random.bit_generator.PCG64")
                            b_new = b_new.replace(b"numpy._core", b"numpy.core")
                            return _orig_cp_loads(b_new, *args, **kwargs)
                        except Exception:
                            pass
                    raise e
            _cp.loads = _cp_loads_shim  # type: ignore
            rospy.loginfo('[rl_algo] Wrapped cloudpickle.loads for NumPy path rewrite')
        except Exception:
            pass
except Exception:
    pass

from stable_baselines3 import DQN

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

        # Create the model (unused when loading a trained model below, but kept for ref)
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
        # Load trained model; path configurable via RL_MODEL_PATH env var
        # Fallback 1: workspace-level highway_dqn/model.zip (from scripts)
        # Fallback 2: package-relative src/models/model.zip
        model_path = os.environ.get("RL_MODEL_PATH")
        if not model_path:
            # Try workspace root
            ws_model = os.path.join("/workspace", "highway_dqn", "model.zip")
            if os.path.isfile(ws_model):
                model_path = ws_model
            else:
                # Try host-mounted path when not in container
                host_ws_model = os.path.join(os.path.expanduser("~"), "ProgrammingProjects", "IMLCA", "highway_dqn", "model.zip")
                if os.path.isfile(host_ws_model):
                    model_path = host_ws_model
                else:
                    # Package bundled model
                    pkg_model = os.path.join(node_path, "src", "models", "model.zip")
                    model_path = pkg_model
    # Note: early NumPy/PCG64 shims are already applied at module import time above.

        # Skip zip patching - it was corrupting pickle data. Rely on runtime shims instead.
        rospy.loginfo(f"[rl_algo] Loading model from: {model_path}")
    # Extra safety left in place: nothing to do here because ctor was already overridden above

        # Replace pickled schedules with simple lambdas and override spaces to avoid unpickling RNG-backed objects
        custom_objs = {
            'lr_schedule': (lambda progress: 5e-4),
            'exploration_schedule': (lambda progress: 0.05),
            'observation_space': env.observation_space,
            'action_space': env.action_space,
        }
        try:
            # Try loading without custom objects first (simpler path)
            self.model = DQN.load(model_path, env=env)
        except Exception as e:
            rospy.logwarn(f"[rl_algo] DQN.load without custom_objects failed: {e}; retrying with custom_objects...")
            try:
                self.model = DQN.load(model_path, env=env, custom_objects=custom_objs)
            except Exception as e2:
                rospy.logwarn(f"[rl_algo] DQN.load with custom_objects failed: {e2}; trying minimal load...")
                try:
                    # Force minimal load - just load the policy network weights, skip optimizer state
                    from stable_baselines3.common.save_util import load_from_zip_file
                    import torch
                    
                    # Load only the basic model data, skip problematic optimizer state
                    data, params, pytorch_variables = load_from_zip_file(
                        model_path, device='cpu', custom_objects=custom_objs
                    )
                    
                    # Create fresh model with same config but don't load optimizer state
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
                    )
                    
                    # Load only the policy network weights
                    if 'policy' in params and hasattr(params['policy'], 'items'):
                        self.model.policy.load_state_dict(params['policy'])
                        rospy.loginfo('[rl_algo] Loaded policy weights only (skipped optimizer state)')
                    else:
                        rospy.logwarn('[rl_algo] No valid policy weights found in model file')
                        
                except Exception as e3:
                    rospy.logerr(f"[rl_algo] All DQN.load attempts failed: {e3}")
                    raise

    def get_action(self, observation):
        action, _states = self.model.predict(observation, deterministic=True)
        return action


class RLAlgoNode:
    def __init__(self, rl: RL):

        self.rl = rl
        
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




# Main function.
if __name__ == '__main__':
    rospy.init_node('scenario_generator')
    try:

        rl = RL()
        scenario_node = RLAlgoNode(rl)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
