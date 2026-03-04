import numpy as np


class TrajectoryMetrics:
    def __init__(self):
        self.timestamps = []
        self.positions = []  # List of np.array (7,)
        self.latencies = []  # List of floats (computation time per step)
        self.clearances = []  # List of floats (min distance to obstacle)
        self.velocities = []  # Derived
        self.accelerations = []  # Derived
        self.jerks = []  # Derived

    def add_data_point(self, timestamp, position, latency, clearance):
        self.timestamps.append(timestamp)
        self.positions.append(position)
        self.latencies.append(latency)
        self.clearances.append(clearance)

    def compute_all(self):
        if len(self.positions) < 4:
            return None

        # 1. Path Length (Euclidean distance in Joint Space)
        path_len = 0.0
        for i in range(1, len(self.positions)):
            path_len += np.linalg.norm(self.positions[i] - self.positions[i - 1])

        # 2. Derivatives (Finite Difference)
        dt_list = np.diff(self.timestamps)
        # Avoid division by zero
        dt_list = np.maximum(dt_list, 1e-6)

        # Velocity: dQ / dt
        self.velocities = []
        for i in range(len(self.positions) - 1):
            dq = self.positions[i + 1] - self.positions[i]
            self.velocities.append(dq / dt_list[i])

        # Acceleration: dV / dt
        self.accelerations = []
        for i in range(len(self.velocities) - 1):
            dv = self.velocities[i + 1] - self.velocities[i]
            self.accelerations.append(dv / dt_list[i])

        # Jerk: dA / dt
        self.jerks = []
        for i in range(len(self.accelerations) - 1):
            da = self.accelerations[i + 1] - self.accelerations[i]
            self.jerks.append(da / dt_list[i])

        # 3. Metrics
        avg_latency = np.mean(self.latencies)
        min_clearance = np.min(self.clearances)

        # Smoothness: Mean Squared Jerk (lower is smoother)
        jerk_magnitudes = [np.linalg.norm(j) for j in self.jerks]
        mean_squared_jerk = np.mean(np.array(jerk_magnitudes) ** 2) if jerk_magnitudes else 0.0

        # Continuity: Max change in velocity (C1 violation check)
        max_vel_jump = 0.0
        if len(self.velocities) > 1:
            vel_diffs = [np.linalg.norm(self.velocities[i] - self.velocities[i - 1]) for i in
                         range(1, len(self.velocities))]
            max_vel_jump = np.max(vel_diffs)

        total_time = self.timestamps[-1] - self.timestamps[0]

        return {
            "total_time": total_time,
            "path_length": path_len,
            "avg_latency": avg_latency,
            "min_clearance": min_clearance,
            "smoothness_jerk": mean_squared_jerk,
            "continuity_max_vel_jump": max_vel_jump
        }