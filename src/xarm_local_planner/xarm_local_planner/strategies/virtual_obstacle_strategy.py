import numpy as np
from collections import deque
from xarm_local_planner.strategies.base_strategy import BaseAPFStrategy


class VirtualObstacleAPFStrategy(BaseAPFStrategy):
    def __init__(self, xi: float, eta: float, rho0: float,
                 vo_eta: float = 0.5, vo_rho0: float = 1.0,
                 stuck_window: int = 50, stuck_threshold: float = 0.05,
                 oscillation_ratio: float = 3.0,
                 goal_tolerance: float = 0.1):

        self.xi = xi
        self.eta = eta
        self.rho0 = rho0

        self.vo_eta = vo_eta
        self.vo_rho0 = vo_rho0
        self.stuck_window = stuck_window
        self.stuck_threshold = stuck_threshold
        self.oscillation_ratio = oscillation_ratio
        self.goal_tolerance = goal_tolerance

        self.position_history = deque(maxlen=self.stuck_window)
        self.virtual_obstacles = []

    def compute_velocity(self, q_curr, q_goal, env_data):
        error = q_curr - q_goal
        tau_att = -self.xi * error

        tau_rep = np.zeros(7)
        if env_data and env_data.data:
            for link_data in env_data.data:
                dist = link_data.min_distance
                if 0 < dist < self.rho0:
                    mag = self.eta * (1.0 / dist - 1.0 / self.rho0) * (1.0 / (dist ** 2))
                    dir_vec = np.array([
                        link_data.direction_vector.x,
                        link_data.direction_vector.y,
                        link_data.direction_vector.z
                    ])
                    norm = np.linalg.norm(dir_vec)
                    if norm > 1e-6:
                        dir_vec = dir_vec / norm

                    F_cart = -1.0 * mag * dir_vec

                    full_jac = np.array(link_data.jacobian)
                    if len(full_jac) > 0:
                        cols = len(full_jac) // 3
                        J = full_jac.reshape(3, cols)
                        J_arm = J[:, :7]
                        tau_link = np.dot(J_arm.T, F_cart)
                        tau_rep += tau_link


        dist_to_goal = np.linalg.norm(error)

        if dist_to_goal > self.goal_tolerance:
            self.position_history.append(q_curr.copy())

            if len(self.position_history) == self.stuck_window:
                net_displacement = np.linalg.norm(self.position_history[-1] - self.position_history[0])
                path_length = sum(np.linalg.norm(self.position_history[i] - self.position_history[i - 1])
                                  for i in range(1, self.stuck_window))

                is_stopped = path_length < self.stuck_threshold
                is_oscillating = (net_displacement < self.stuck_threshold) and \
                                 (path_length > self.oscillation_ratio * net_displacement)

                if is_stopped or is_oscillating:
                    center_q = np.mean(self.position_history, axis=0)
                    self.virtual_obstacles.append(center_q)
                    reason = "Oscillation" if is_oscillating else "Stopped"
                    print(
                        f"[VirtualObstacleStrategy] {reason} detected! Dropped Virtual Obstacle #{len(self.virtual_obstacles)}")
                    self.position_history.clear()
        else:
            self.position_history.clear()

        tau_vo = np.zeros(7)
        for q_vo in self.virtual_obstacles:
            vo_dist = np.linalg.norm(q_curr - q_vo)
            if 0 < vo_dist < self.vo_rho0:
                mag = self.vo_eta * (1.0 / vo_dist - 1.0 / self.vo_rho0) * (1.0 / (vo_dist ** 2))
                dir_vec = (q_curr - q_vo) / vo_dist
                tau_vo += mag * dir_vec

        return tau_att + tau_rep + tau_vo