import numpy as np
from collections import deque
from xarm_local_planner.strategies.base_strategy import BaseAPFStrategy


class HybridVortexVOStrategy(BaseAPFStrategy):
    def __init__(self, xi: float, eta: float, rho0: float,
                 vo_eta: float = 0.5, vo_rho0: float = 1.0,
                 epsilon: float = 0.5, xi_max_multiplier: float = 3.0,
                 goal_tolerance: float = 0.05):
        self.xi = xi
        self.eta = eta
        self.rho0 = rho0
        self.vo_eta = vo_eta
        self.vo_rho0 = vo_rho0
        self.epsilon = epsilon
        self.xi_max_multiplier = xi_max_multiplier
        self.goal_tolerance = goal_tolerance

        self.stuck_window = 25
        self.stuck_threshold = 0.1
        self.oscillation_ratio = 3.0
        self.position_history = deque(maxlen=self.stuck_window)
        self.virtual_obstacles = []

    def compute_velocity(self, q_curr, q_goal, env_data):
        error_q = q_goal - q_curr
        dist_to_goal = np.linalg.norm(error_q)

        adaptive_scale = 1.0
        if dist_to_goal < 0.5:
            adaptive_scale = 1.0 + (self.xi_max_multiplier - 1.0) * (1.0 - dist_to_goal / 0.5)
            adaptive_scale = np.clip(adaptive_scale, 1.0, self.xi_max_multiplier)

        xi_effective = self.xi * adaptive_scale
        tau_att = xi_effective * error_q

        repulsion_scale = 1.0
        if dist_to_goal < self.goal_tolerance:
            repulsion_scale = dist_to_goal / self.goal_tolerance
        elif dist_to_goal < (self.goal_tolerance * 2):
            repulsion_scale = dist_to_goal / (self.goal_tolerance * 2)

        tau_rep = np.zeros(7)
        if env_data and env_data.data:
            for link_data in env_data.data:
                dist = link_data.min_distance
                if 0 < dist < self.rho0:
                    n = np.array(
                        [link_data.direction_vector.x, link_data.direction_vector.y, link_data.direction_vector.z])
                    norm_n = np.linalg.norm(n)
                    if norm_n > 1e-6: n /= norm_n

                    full_jac = np.array(link_data.jacobian)
                    if len(full_jac) == 0: continue
                    J = full_jac.reshape(3, len(full_jac) // 3)[:, :7]

                    v_goal = np.dot(J, error_q)
                    v_goal_norm = np.linalg.norm(v_goal)
                    if v_goal_norm > 1e-6: v_goal /= v_goal_norm

                    v_tangent = v_goal - np.dot(v_goal, n) * n
                    v_tan_norm = np.linalg.norm(v_tangent)
                    if v_tan_norm > 1e-6: v_tangent /= v_tan_norm

                    mag_rep = self.eta * (1.0 / dist - 1.0 / self.rho0) * (1.0 / (dist ** 2))
                    F_normal = -1.0 * mag_rep * n * repulsion_scale
                    F_slide = self.epsilon * mag_rep * v_tangent * repulsion_scale
                    tau_rep += np.dot(J.T, F_normal + F_slide)

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