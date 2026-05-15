import numpy as np
from collections import deque
from xarm_local_planner.strategies.base_strategy import BaseAPFStrategy


class VirtualHillsAPFStrategy(BaseAPFStrategy):
    def __init__(self, xi: float, eta: float, rho0: float,
                 hill_height: float = 2.0, hill_sigma: float = 0.2,
                 stuck_window: int = 50,
                 stuck_threshold: float = 0.02,
                 variance_threshold: float = 0.8,
                 goal_tolerance: float = 0.1,
                 xi_max_multiplier: float = 3.0):

        self.xi = xi
        self.eta = eta
        self.rho0 = rho0

        self.hill_height = hill_height
        self.hill_sigma = hill_sigma

        self.stuck_window = stuck_window
        self.stuck_threshold = stuck_threshold
        self.variance_threshold = variance_threshold
        self.goal_tolerance = goal_tolerance
        self.xi_max_multiplier = xi_max_multiplier

        self.force_history = deque(maxlen=self.stuck_window)
        self.position_history = deque(maxlen=self.stuck_window)
        self.virtual_hills = []

    def compute_velocity(self, q_curr, q_goal, env_data):
        error = q_curr - q_goal
        dist_to_goal = np.linalg.norm(error)

        adaptive_scale = 1.0
        if dist_to_goal < 0.5:
            adaptive_scale = 1.0 + (self.xi_max_multiplier - 1.0) * (1.0 - dist_to_goal / 0.5)
            adaptive_scale = np.clip(adaptive_scale, 1.0, self.xi_max_multiplier)

        xi_effective = self.xi * adaptive_scale
        tau_att = -xi_effective * error

        tau_rep = np.zeros(7)
        if env_data and env_data.data:
            for link_data in env_data.data:
                dist = link_data.min_distance
                if 0 < dist < self.rho0:
                    mag = self.eta * (1.0 / dist - 1.0 / self.rho0) * (1.0 / (dist ** 2))
                    dir_vec = np.array(
                        [link_data.direction_vector.x, link_data.direction_vector.y, link_data.direction_vector.z])
                    norm = np.linalg.norm(dir_vec)
                    if norm > 1e-6:
                        dir_vec /= norm

                    F_cart = -1.0 * mag * dir_vec
                    full_jac = np.array(link_data.jacobian)
                    if len(full_jac) > 0:
                        J = full_jac.reshape(3, len(full_jac) // 3)
                        tau_rep += np.dot(J[:, :7].T, F_cart)

        tau_net = tau_att + tau_rep

        if dist_to_goal > self.goal_tolerance:
            force_norm = np.linalg.norm(tau_net)
            if force_norm > 1e-9:
                self.force_history.append(tau_net / force_norm)

            self.position_history.append(q_curr.copy())

            if len(self.position_history) == self.stuck_window:
                alignment_score = 1.0
                if len(self.force_history) == self.stuck_window:
                    avg_direction = np.mean(self.force_history, axis=0)
                    alignment_score = np.linalg.norm(avg_direction)

                is_oscillating = alignment_score < (1.0 - self.variance_threshold)

                path_length = sum(np.linalg.norm(self.position_history[i] - self.position_history[i - 1])
                                  for i in range(1, len(self.position_history)))
                is_stopped = path_length < self.stuck_threshold

                if is_oscillating or is_stopped:
                    self.virtual_hills.append(q_curr.copy())
                    self.force_history.clear()
                    self.position_history.clear()
        else:
            self.force_history.clear()
            self.position_history.clear()

        tau_hills = np.zeros(7)
        for q_hill in self.virtual_hills:
            diff = q_curr - q_hill
            d_sq = np.sum(diff ** 2)
            magnitude = (self.hill_height / (self.hill_sigma ** 2)) * np.exp(-d_sq / (2 * self.hill_sigma ** 2))
            tau_hills += magnitude * diff

        return tau_net + tau_hills