import numpy as np
from xarm_local_planner.strategies.base_strategy import BaseAPFStrategy


class GuidedVortexStrategy(BaseAPFStrategy):
    def __init__(self, xi: float, eta: float, rho0: float, epsilon: float,
                 xi_max_multiplier: float, goal_tolerance: float):
        self.xi = xi
        self.eta = eta
        self.rho0 = rho0
        self.epsilon = epsilon
        self.xi_max_multiplier = xi_max_multiplier
        self.goal_tolerance = goal_tolerance

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

        return tau_att + tau_rep