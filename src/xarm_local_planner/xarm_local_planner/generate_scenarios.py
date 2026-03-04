import json
import random
import math


def get_random_orientation():
    """
    Generates a target orientation (Roll, Pitch, Yaw).
    Base is [3.14, 0, 0] (pointing down).
    We add full random Yaw and slight tilts to Roll/Pitch to vary the grasp.
    """
    roll = 3.14
    pitch = 0.0
    yaw = random.uniform(-3.14, 3.14)  # Full rotation around Z

    # Slight perturbation for realism
    roll += random.uniform(-0.15, 0.15)
    pitch += random.uniform(-0.15, 0.15)

    return [roll, pitch, yaw]


def get_random_pose(min_r=0.35, max_r=0.6):
    """
    Generates a reachable target pose in front of the robot.
    """
    r = random.uniform(min_r, max_r)
    theta = random.uniform(-1.0, 1.0)  # +/- ~60 degrees
    z = random.uniform(0.15, 0.45)  # Safe height

    x = r * math.cos(theta)
    y = r * math.sin(theta)

    rpy = get_random_orientation()
    return [x, y, z, rpy[0], rpy[1], rpy[2]]


def distance(p1, p2):
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2 + (p1[2] - p2[2]) ** 2)


def generate_scenarios():
    scenarios = []

    # Standard "Home" joints (Retracted)
    start_joints = [0.0, -0.5, 0.0, 0.0, 0.0, 0.0, 0.0]
    # Heuristic start position (End Effector approx location)
    start_pos_heuristic = [0.207, 0.0, 0.3]

    total_scenarios = 100

    for i in range(total_scenarios):
        target_pose = get_random_pose()
        tx, ty, tz = target_pose[:3]

        scenario = {
            "id": i,
            "start_joints": start_joints,
            "target_pose": target_pose,
            "obstacles": [],
            "difficulty": "easy"
        }

        # Calculate path vector and length
        path_vec = [tx - start_pos_heuristic[0], ty - start_pos_heuristic[1], tz - start_pos_heuristic[2]]
        path_len = math.sqrt(path_vec[0] ** 2 + path_vec[1] ** 2 + path_vec[2] ** 2)

        # Minimum safe distance from target (0.06 requested + obstacle radius buffer)
        # We ensure the obstacle center is at least this far from the target center
        SAFE_DIST = 0.06

        # --- TYPE 1: SIMPLE PATH OBSTACLE (10%) ---
        # Replaces "Free Space". Now puts a simple sphere on the path.
        if i < 10:
            scenario["difficulty"] = "easy"

            # Place roughly in the middle (30% to 70% of path)
            t = random.uniform(0.3, 0.7)

            # Ensure it respects the 0.06m buffer from target
            # Distance from target is (1-t)*path_len
            dist_from_target = (1.0 - t) * path_len

            # If too close, pull it back
            if dist_from_target < SAFE_DIST + 0.05:  # 0.05 is approx radius
                t = 1.0 - ((SAFE_DIST + 0.06) / path_len)
                if t < 0.2: t = 0.2  # Don't push it too close to start either

            ox = start_pos_heuristic[0] + path_vec[0] * t
            oy = start_pos_heuristic[1] + path_vec[1] * t
            oz = start_pos_heuristic[2] + path_vec[2] * t

            scenario["obstacles"].append({
                "type": "sphere",
                "pos": [ox, oy, oz],
                "radius": 0.05
            })

        # --- TYPE 2: VARIED BLOCKER (30%) ---
        # Standard blocking object, but strictly checked for distance
        elif i < 40:
            scenario["difficulty"] = "medium"

            # Try to place it in the "sweet spot" for blocking (40-60%)
            t = random.uniform(0.4, 0.6)

            # Enforce Safe Distance Logic
            # 1. Check Target Distance
            dist_from_target = (1.0 - t) * path_len
            if dist_from_target < (SAFE_DIST + 0.1):  # 0.1 buffer for larger boxes
                t = 1.0 - ((SAFE_DIST + 0.12) / path_len)

            # 2. Check Start Distance
            dist_from_start = t * path_len
            if dist_from_start < 0.1:
                t = 0.1 / path_len  # Move at least 10cm from start

            ox = start_pos_heuristic[0] + path_vec[0] * t
            oy = start_pos_heuristic[1] + path_vec[1] * t
            oz = start_pos_heuristic[2] + path_vec[2] * t

            obs_type = random.choice(["sphere", "box", "cylinder"])

            if obs_type == "sphere":
                scenario["obstacles"].append({
                    "type": "sphere",
                    "pos": [ox, oy, oz],
                    "radius": random.uniform(0.06, 0.09)
                })
            elif obs_type == "box":
                scenario["obstacles"].append({
                    "type": "box",
                    "pos": [ox, oy, oz],
                    "dim": [0.05, 0.2, 0.2]
                })
            elif obs_type == "cylinder":
                scenario["obstacles"].append({
                    "type": "cylinder",
                    "pos": [ox, oy, oz],
                    "radius": 0.05,
                    "height": 0.3
                })

        # --- TYPE 3: THE FOREST (20%) ---
        elif i < 60:
            scenario["difficulty"] = "hard"
            num_trees = random.randint(3, 5)

            for _ in range(num_trees):
                ox = random.uniform(0.2, 0.5)
                oy = random.uniform(-0.3, 0.3)
                oz = random.uniform(0.1, 0.3)

                obs_pos = [ox, oy, oz]
                # Check distances
                if distance(obs_pos, [tx, ty, tz]) > (SAFE_DIST + 0.05) and \
                        distance(obs_pos, start_pos_heuristic) > 0.15:
                    scenario["obstacles"].append({
                        "type": "cylinder",
                        "pos": [ox, oy, oz],
                        "radius": 0.03,
                        "height": 0.4
                    })

        # --- TYPE 4: THE WINDOW (20%) ---
        elif i < 80:
            scenario["difficulty"] = "hard"

            mx = (start_pos_heuristic[0] + tx) / 2.0
            my = (start_pos_heuristic[1] + ty) / 2.0
            mz = (start_pos_heuristic[2] + tz) / 2.0

            # Ensure the "window wall" isn't too close to the target
            if distance([mx, my, mz], [tx, ty, tz]) < (SAFE_DIST + 0.05):
                # If too close, move midpoint back towards start
                mx = start_pos_heuristic[0] + path_vec[0] * 0.4
                my = start_pos_heuristic[1] + path_vec[1] * 0.4
                mz = start_pos_heuristic[2] + path_vec[2] * 0.4

            window_size = 0.15
            wall_thickness = 0.02

            scenario["obstacles"].append({
                "type": "box",
                "pos": [mx, my, mz + window_size + 0.1],
                "dim": [wall_thickness, 0.5, 0.2]
            })
            scenario["obstacles"].append({
                "type": "box",
                "pos": [mx, my, mz - window_size - 0.1],
                "dim": [wall_thickness, 0.5, 0.2]
            })

        # --- TYPE 5: THE SHELF / LOCAL MINIMA TRAP (20%) ---
        else:
            scenario["difficulty"] = "hard"
            # Standard shelf logic
            # Floor
            scenario["obstacles"].append({
                "type": "box",
                "pos": [tx, ty, tz - 0.12],
                "dim": [0.2, 0.2, 0.02]
            })
            # Ceiling
            scenario["obstacles"].append({
                "type": "box",
                "pos": [tx, ty, tz + 0.12],
                "dim": [0.2, 0.2, 0.02]
            })
            # Back wall (behind target)
            vec_x = tx
            vec_y = ty
            mag = math.sqrt(vec_x ** 2 + vec_y ** 2)
            if mag > 0:
                dir_x = vec_x / mag
                dir_y = vec_y / mag
                wall_dist = 0.12
                scenario["obstacles"].append({
                    "type": "box",
                    "pos": [tx + dir_x * wall_dist, ty + dir_y * wall_dist, tz],
                    "dim": [0.02, 0.2, 0.25]
                })

        scenarios.append(scenario)

    with open("scenarios.json", "w") as f:
        json.dump(scenarios, f, indent=4)
    print(f"Generated {total_scenarios} scenarios in scenarios.json")


if __name__ == "__main__":
    generate_scenarios()