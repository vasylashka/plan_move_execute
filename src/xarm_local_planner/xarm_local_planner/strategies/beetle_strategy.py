import numpy as np
from collections import deque
from xarm_local_planner.strategies.base_strategy import BaseAPFStrategy


class DefaultBeetleStrategy(BaseAPFStrategy):
    def __init__(self, xi: float, eta: float, rho0: float,
                 bas_d: float = 0.25, bas_steps: int = 50,
                 stuck_threshold: float = 0.02,
                 variance_threshold: float = 0.8,
                 goal_tolerance: float = 0.1):

        self.xi = xi
        self.eta = eta
        self.rho0 = rho0

        # Параметри Beetle Antennae Search
        self.bas_d = bas_d
        self.bas_recovery_steps = bas_steps
        self.current_recovery_step = 0

        # Параметри детекції Stuck + Oscillation
        self.goal_tolerance = goal_tolerance
        self.stuck_threshold = stuck_threshold
        self.variance_threshold = variance_threshold
        self.stuck_window = 50

        self.position_history = deque(maxlen=self.stuck_window)
        self.force_history = deque(maxlen=self.stuck_window)  # Історія векторів сил

    def _get_potential(self, q, q_goal, env_data):
        """Оцінка потенціалу для BAS"""
        dist_to_goal = np.linalg.norm(q - q_goal)
        u_att = 0.5 * self.xi * (dist_to_goal ** 2)

        u_rep = 0.0
        if env_data and env_data.data:
            for link_data in env_data.data:
                dist = link_data.min_distance
                if 0 < dist < self.rho0:
                    u_rep += 0.5 * self.eta * (1.0 / dist - 1.0 / self.rho0) ** 2
        return u_att + u_rep

    def compute_velocity(self, q_curr, q_goal, env_data):
        error_q = q_goal - q_curr
        dist_to_goal = np.linalg.norm(error_q)

        # --- Розрахунок стандартного APF вектора (для детекції осциляцій) ---
        tau_att = self.xi * error_q
        tau_rep = np.zeros(7)

        if env_data and env_data.data:
            for link_data in env_data.data:
                dist = link_data.min_distance
                if 0 < dist < self.rho0:
                    mag = self.eta * (1.0 / dist - 1.0 / self.rho0) * (1.0 / (dist ** 2))
                    dir_vec = np.array(
                        [link_data.direction_vector.x, link_data.direction_vector.y, link_data.direction_vector.z])
                    norm = np.linalg.norm(dir_vec)
                    if norm > 1e-6: dir_vec /= norm

                    F_cart = -1.0 * mag * dir_vec
                    full_jac = np.array(link_data.jacobian)
                    if len(full_jac) > 0:
                        J = full_jac.reshape(3, len(full_jac) // 3)[:, :7]
                        tau_rep += np.dot(J.T, F_cart)

        tau_net = tau_att + tau_rep

        # --- Детекція осциляцій та застрягання ---
        if dist_to_goal > self.goal_tolerance:
            # Зберігаємо нормалізований вектор сили для аналізу стабільності
            net_norm = np.linalg.norm(tau_net)
            if net_norm > 1e-9:
                self.force_history.append(tau_net / net_norm)

            self.position_history.append(q_curr.copy())

            if len(self.position_history) == self.stuck_window:
                # 1. Перевірка на зупинку (Stuck)
                path_len = sum(np.linalg.norm(self.position_history[i] - self.position_history[i - 1])
                               for i in range(1, self.stuck_window))
                is_stopped = path_len < self.stuck_threshold

                # 2. Перевірка на осциляцію (Alignment)
                # Якщо середня сума векторів мала — вектори "б'ються" один об одного
                alignment_score = 1.0
                if len(self.force_history) == self.stuck_window:
                    avg_force_direction = np.mean(self.force_history, axis=0)
                    alignment_score = np.linalg.norm(avg_force_direction)

                is_oscillating = alignment_score < (1.0 - self.variance_threshold)

                if (is_stopped or is_oscillating) and self.current_recovery_step == 0:
                    self.current_recovery_step = self.bas_recovery_steps
                    reason = "Oscillation" if is_oscillating else "Stuck"
                    print(f"[BAS Recovery] {reason} detected! (Alignment: {alignment_score:.2f}). Starting search...")
                    self.force_history.clear()
                    self.position_history.clear()
        else:
            self.force_history.clear()
            self.position_history.clear()

        # --- Вибір режиму руху ---
        if self.current_recovery_step > 0:
            self.current_recovery_step -= 1
            b = np.random.randn(7)
            b /= np.linalg.norm(b)
            f_l = self._get_potential(q_curr + self.bas_d * b, q_goal, env_data)
            f_r = self._get_potential(q_curr - self.bas_d * b, q_goal, env_data)
            return np.sign(f_l - f_r) * b * 0.5

        return tau_net