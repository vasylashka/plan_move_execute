import numpy as np
from xarm_local_planner.strategies.base_strategy import BaseAPFStrategy

class VortexAPFStrategy(BaseAPFStrategy):
    def __init__(self, xi: float, eta: float, rho0: float, gamma: float, xi_max_multiplier: float = 3.0):
        self.xi = xi
        self.eta = eta
        self.rho0 = rho0
        self.gamma = gamma
        self.xi_max_multiplier = xi_max_multiplier # Максимальний множник підсилення

    def compute_velocity(self, q_curr, q_goal, env_data):
        # 1. Адаптивне притягання (Dynamic Attraction Gain)
        error = q_curr - q_goal
        dist_to_goal = np.linalg.norm(error)

        # Логіка як у Virtual Hills: підсилюємо xi, якщо дистанція менша за 0.5 рад
        adaptive_scale = 1.0
        if dist_to_goal < 0.5:
            adaptive_scale = 1.0 + (self.xi_max_multiplier - 1.0) * (1.0 - dist_to_goal / 0.5)
            adaptive_scale = np.clip(adaptive_scale, 1.0, self.xi_max_multiplier)

        xi_effective = self.xi * adaptive_scale
        tau_att = -xi_effective * error

        # 2. Сила відштовхування та вихрова компонента (без змін)
        tau_rep = np.zeros(7)

        if env_data and env_data.data:
            for link_data in env_data.data:
                dist = link_data.min_distance
                if 0 < dist < self.rho0:
                    mag = self.eta * (1.0 / dist - 1.0 / self.rho0) * (1.0 / (dist ** 2))

                    dir_to_obs = np.array([
                        link_data.direction_vector.x,
                        link_data.direction_vector.y,
                        link_data.direction_vector.z
                    ])
                    norm = np.linalg.norm(dir_to_obs)
                    if norm > 1e-6:
                        dir_to_obs /= norm

                    # Вихровий вектор (ковзання)
                    vortex_vec = np.cross(dir_to_obs, np.array([0, 0, 1]))
                    v_norm = np.linalg.norm(vortex_vec)
                    if v_norm > 1e-6:
                        vortex_vec /= v_norm

                    F_cart = mag * (-1.0 * dir_to_obs + self.gamma * vortex_vec)

                    full_jac = np.array(link_data.jacobian)
                    if len(full_jac) > 0:
                        J = full_jac.reshape(3, len(full_jac) // 3)
                        tau_rep += np.dot(J[:, :7].T, F_cart)

        return tau_att + tau_rep