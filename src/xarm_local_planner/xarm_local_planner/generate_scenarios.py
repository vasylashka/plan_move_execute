import json
import random
import math


def distance(p1, p2):
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2 + (p1[2] - p2[2]) ** 2)


def is_point_in_box(p, box_pos, box_dim, buffer):
    """Checks if a point is within a box volume plus a safety buffer."""
    for i in range(3):
        if not (box_pos[i] - box_dim[i] / 2 - buffer <= p[i] <= box_pos[i] + box_dim[i] / 2 + buffer):
            return False
    return True


def is_obstacle_safe(obs_pos, obs_dim_or_radius, target_pos, is_box=True):
    """
    Improved safety check using geometric-specific logic.
    Ensures start/goal points are not inside or too close to obstacles.
    """
    start_ee = [0.207, 0.0, 0.315]
    buffer = 0.08  # 8cm clearance buffer

    if is_box:
        # Check if Start or Goal is inside the box's expanded bounding box
        if is_point_in_box(start_ee, obs_pos, obs_dim_or_radius, buffer):
            return False
        if is_point_in_box(target_pos[:3], obs_pos, obs_dim_or_radius, buffer):
            return False
    else:
        # Sphere safety check
        if distance(obs_pos, start_ee) < (obs_dim_or_radius + buffer):
            return False
        if distance(obs_pos, target_pos[:3]) < (obs_dim_or_radius + buffer):
            return False

    # Prevent spawning inside the robot's central pedestal
    dist_xy = math.sqrt(obs_pos[0] ** 2 + obs_pos[1] ** 2)
    if dist_xy < 0.16 and obs_pos[2] < 0.4:
        return False

    return True


def get_smart_pose(orientation_type="downward", min_radius=0.35):
    """Generates a reachable target pose in the front workspace."""
    r = random.uniform(min_radius, 0.55)
    theta = random.uniform(-0.8, 0.8)
    z = random.uniform(0.15, 0.45)

    x = r * math.cos(theta)
    y = r * math.sin(theta)

    if orientation_type == "horizontal":
        rpy = [3.1415, -1.5707, 0.0]
    else:
        yaw = random.uniform(-3.14, 3.14)
        rpy = [3.1415 + random.uniform(-0.1, 0.1),
               random.uniform(-0.1, 0.1),
               yaw]
    return [x, y, z, rpy[0], rpy[1], rpy[2]]


def generate_scenarios():
    scenarios = []
    start_joints = [0.0, -0.5, 0.0, 0.0, 0.0, 0.0, 0.0]
    start_pos_ee = [0.207, 0.0, 0.315]
    total_scenarios = 100

    for i in range(total_scenarios):
        scenario = {
            "id": i,
            "start_joints": start_joints,
            "target_pose": [],
            "obstacles": [],
            "difficulty": "medium",
            "scenario_type": ""
        }

        # --- CATEGORY 1: FREE SPACE (0-19) ---
        if i < 20:
            scenario["scenario_type"] = "free_space"
            while not scenario["obstacles"]:
                scenario["target_pose"] = get_smart_pose()
                tx, ty, tz = scenario["target_pose"][:3]
                mx, my, mz = (start_pos_ee[0] + tx) / 2, (start_pos_ee[1] + ty) / 2 + 0.15, (start_pos_ee[2] + tz) / 2
                if is_obstacle_safe([mx, my, mz], 0.05, scenario["target_pose"], False):
                    scenario["obstacles"].append({"type": "sphere", "pos": [mx, my, mz], "radius": 0.05})

        # --- CATEGORY 2: COLLINEAR TRAP (20-39) ---
        elif i < 40:
            scenario["scenario_type"] = "collinear_trap"
            # Require a farther target to fit the trap sphere safely
            while not scenario["obstacles"]:
                scenario["target_pose"] = get_smart_pose(min_radius=0.45)
                tx, ty, tz = scenario["target_pose"][:3]

                # Place obstacle with a tiny Y-offset to break symmetry for the APF
                mx, my, mz = (start_pos_ee[0] + tx) / 2, (start_pos_ee[1] + ty) / 2 + 0.015, (start_pos_ee[2] + tz) / 2
                if is_obstacle_safe([mx, my, mz], 0.06, scenario["target_pose"], False):
                    scenario["obstacles"].append({"type": "sphere", "pos": [mx, my, mz], "radius": 0.06})

        # --- CATEGORY 3: WALL WINDOW (40-59) ---
        elif i < 60:
            scenario["scenario_type"] = "wall_window"
            while not scenario["obstacles"]:
                scenario["target_pose"] = get_smart_pose(min_radius=0.48)
                tx, ty, tz = scenario["target_pose"][:3]

                mx = (start_pos_ee[0] + tx) / 2.0
                my = (start_pos_ee[1] + ty) / 2.0
                mz = (start_pos_ee[2] + tz) / 2.0
                gap = 0.22  # Increased for xArm7 clearance
                dim = [0.05, 0.4, 0.5]

                obs1_pos = [mx, my - gap / 2 - 0.2, mz]
                obs2_pos = [mx, my + gap / 2 + 0.2, mz]

                if is_obstacle_safe(obs1_pos, dim, scenario["target_pose"]) and \
                        is_obstacle_safe(obs2_pos, dim, scenario["target_pose"]):
                    scenario["obstacles"].append({"type": "box", "pos": obs1_pos, "dim": dim})
                    scenario["obstacles"].append({"type": "box", "pos": obs2_pos, "dim": dim})

        # --- CATEGORY 4: SHELF TRAP (60-79) ---
        elif i < 80:
            scenario["scenario_type"] = "u_shape_shelf"
            scenario["target_pose"] = get_smart_pose(orientation_type="horizontal")
            tx, ty, tz = scenario["target_pose"][:3]
            scenario["obstacles"].append({"type": "box", "pos": [tx + 0.1, ty, tz - 0.15], "dim": [0.3, 0.4, 0.02]})
            scenario["obstacles"].append({"type": "box", "pos": [tx + 0.1, ty, tz + 0.15], "dim": [0.3, 0.4, 0.02]})
            scenario["obstacles"].append({"type": "box", "pos": [tx + 0.25, ty, tz], "dim": [0.02, 0.4, 0.3]})

        # --- CATEGORY 5: FOREST CLUTTER (80-99) ---
        else:
            scenario["scenario_type"] = "forest_clutter"
            scenario["target_pose"] = get_smart_pose()
            while len(scenario["obstacles"]) < 5:
                ox, oy, oz = random.uniform(0.2, 0.6), random.uniform(-0.4, 0.4), random.uniform(0.1, 0.5)
                if is_obstacle_safe([ox, oy, oz], 0.04, scenario["target_pose"], False):
                    scenario["obstacles"].append({"type": "sphere", "pos": [ox, oy, oz], "radius": 0.04})

        scenarios.append(scenario)

    with open("scenarios.json", "w") as f:
        json.dump(scenarios, f, indent=4)
    print(f"Generated {len(scenarios)} guaranteed scenarios in scenarios.json")


if __name__ == "__main__":
    generate_scenarios()