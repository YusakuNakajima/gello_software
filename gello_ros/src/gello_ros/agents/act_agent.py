from typing import Any, Dict, Protocol
import numpy as np
from gello_ros.agents.agent import Agent


class ACTAgent(Agent):
    def __init__(self, policy, camera_names):
        self.policy = policy
        self.camera_names = camera_names

    def act(self, obs: Dict[str, Any]) -> np.ndarray:
        name = self.camera_names[0]
        all_actions = self.policy(obs["joint_positions"], obs[f"{name}_rgb"])
        print(f"all_actions: {all_actions}")
        return all_actions
