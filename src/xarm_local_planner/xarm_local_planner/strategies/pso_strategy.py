import numpy as np
from xarm_local_planner.strategies.base_strategy import BaseAPFStrategy


class PsoOptimizedStrategy(BaseAPFStrategy):
    def __init__(self, rho0, num_particles=12, iterations=5):
        self.rho0 = rho0
        self.num_particles = num_particles
        self.iterations = iterations

        # Простір пошуку: [attraction_gain (0.1-5.0), repulsion_gain (0.001-0.1)]
        self.bounds = np.array([[0.5, 5.0], [0.001, 0.05]])

        # Ініціалізація часток
        self.particles = np.random.uniform(self.bounds[:, 0], self.bounds[:, 1], (self.num_particles, 2))
        self.velocities = np.zeros((self.num_particles, 2))
        self.p_best = self.particles.copy()
        self.p_best_scores = np.full(self.num_particles, np.inf)
        self.g_best = self.particles[0].copy()
        self.g_best_score = np.inf

    def _calc_apf_vector(self, q, q_goal, env_data, xi, eta):
        """Базовий розрахунок APF без сторонніх ефектів для оцінки fitness"""
        tau_att = -xi * (q - q_goal)
        tau_rep = np.zeros(7)
        if env_data and env_data.data:
            for link in env_data.data:
                d = link.min_distance
                if 0 < d < self.rho0:
                    mag = eta * (1.0 / d - 1.0 / self.rho0) * (1.0 / (d ** 2))
                    n = np.array([link.direction_vector.x, link.direction_vector.y, link.direction_vector.z])
                    full_jac = np.array(link.jacobian)
                    if len(full_jac) > 0:
                        J = full_jac.reshape(3, len(full_jac) // 3)[:, :7]
                        tau_rep += np.dot(J.T, -1.0 * mag * (n / np.linalg.norm(n)))
        return tau_att + tau_rep

    def _evaluate_fitness(self, params, q_curr, q_goal, env_data):
        xi, eta = params
        tau = self._calc_apf_vector(q_curr, q_goal, env_data, xi, eta)

        # Прогноз наступного стану (Forward Euler)
        dt = 0.02
        q_next = q_curr + np.clip(tau, -1.0, 1.0) * dt

        dist_goal = np.linalg.norm(q_next - q_goal)

        # Штраф за близькість до перешкод
        collision_penalty = 0.0
        if env_data and env_data.data:
            min_d = min([l.min_distance for l in env_data.data])
            if min_d < 0.02:  # Критична відстань 2см
                collision_penalty = 10.0 / (min_d + 1e-4)

        # Важливо: додаємо штраф за занадто великі зусилля (стабільність)
        effort_penalty = 0.1 * np.linalg.norm(tau)

        return dist_goal + collision_penalty + effort_penalty

    def compute_velocity(self, q_curr, q_goal, env_data):
        # Алгоритм PSO (Particle Swarm Optimization)
        w, c1, c2 = 0.5, 1.5, 1.5  # Параметри інерції та навчання

        for _ in range(self.iterations):
            for i in range(self.num_particles):
                score = self._evaluate_fitness(self.particles[i], q_curr, q_goal, env_data)

                if score < self.p_best_scores[i]:
                    self.p_best_scores[i] = score
                    self.p_best[i] = self.particles[i].copy()

                if score < self.g_best_score:
                    self.g_best_score = score
                    self.g_best = self.particles[i].copy()

            # Оновлення часток
            r1, r2 = np.random.rand(self.num_particles, 2), np.random.rand(self.num_particles, 2)
            self.velocities = (w * self.velocities +
                               c1 * r1 * (self.p_best - self.particles) +
                               c2 * r2 * (self.g_best - self.particles))

            self.particles = np.clip(self.particles + self.velocities, self.bounds[:, 0], self.bounds[:, 1])

        # Повертаємо вектор швидкості з найкращими знайденими параметрами
        best_xi, best_eta = self.g_best
        return self._calc_apf_vector(q_curr, q_goal, env_data, best_xi, best_eta)