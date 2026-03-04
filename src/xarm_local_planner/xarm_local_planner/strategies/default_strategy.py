import numpy as np
from xarm_local_planner.strategies.base_strategy import BaseAPFStrategy


class DefaultAPFStrategy(BaseAPFStrategy):
    def __init__(self, xi: float, eta: float, rho0: float):
        self.xi = xi  # Attraction gain
        self.eta = eta  # Repulsion gain
        self.rho0 = rho0  # Influence distance (meters)

    def compute_velocity(self, q_curr, q_goal, env_data):
        # 1. Attraction Force (Pull to goal)
        error = q_curr - q_goal
        tau_att = -self.xi * error

        # 2. Repulsion Force (Push from obstacles)
        tau_rep = np.zeros(7)

        if env_data and env_data.data:
            for link_data in env_data.data:
                dist = link_data.min_distance

                # Only apply force if within influence distance
                if 0 < dist < self.rho0:
                    mag = self.eta * (1.0 / dist - 1.0 / self.rho0) * (1.0 / (dist ** 2))

                    dir_vec = np.array([
                        link_data.direction_vector.x,
                        link_data.direction_vector.y,
                        link_data.direction_vector.z
                    ])

                    # Normalize the direction vector
                    norm = np.linalg.norm(dir_vec)
                    if norm > 1e-6:
                        dir_vec = dir_vec / norm

                    F_cart = -1.0 * mag * dir_vec

                    # Map Cartesian force to Joint torque using the Jacobian
                    full_jac = np.array(link_data.jacobian)
                    if len(full_jac) > 0:
                        cols = len(full_jac) // 3
                        J = full_jac.reshape(3, cols)
                        J_arm = J[:, :7]  # Use first 7 joints of the xArm7

                        tau_link = np.dot(J_arm.T, F_cart)
                        tau_rep += tau_link

        return tau_att + tau_rep