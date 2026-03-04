from abc import ABC, abstractmethod
import numpy as np


class BaseAPFStrategy(ABC):
    """
    Abstract Base Class for all APF-based steering strategies.
    All subclasses MUST implement compute_velocity.
    """

    @abstractmethod
    def compute_velocity(self, q_curr: np.ndarray, q_goal: np.ndarray, env_data) -> np.ndarray:
        """
        Calculate the joint velocity vector based on the current state and environment.

        :param q_curr: Current joint positions (7,)
        :param q_goal: Target joint positions (7,)
        :param env_data: The result from /planning/get_apf_distances service
        :return: A numpy array of joint velocities (7,) or None if calculation fails.
        """
        pass