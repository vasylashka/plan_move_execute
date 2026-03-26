import json
import random
import math


def distance(p1, p2):
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2 + (p1[2] - p2[2]) ** 2)


def get_smart_pose(orientation_type="downward"):
    """
    Generates a reachable target pose constrained strictly to the front workspace
    to guarantee kinematic reachability.
    """
    # Constrain to the safe front workspace to avoid self-collisions
    r = random.uniform(0.35, 0.55)
    theta = random.uniform(-0.8, 0.8)  # Keep it in front (roughly +/- 45 degrees)
    z = random.uniform(0.15, 0.45)

    x = r * math.cos(theta)
    y = r * math.sin(theta)

    if orientation_type == "horizontal":
        # Gripper points forward (along +X axis). Ideal for inserting into shelves.
        # Assuming standard xArm URDF: Roll=Pi, Pitch=-Pi/2, Yaw=0
        rpy = [3.1415, -1.5707, 0.0]
    else:
        # Gripper points downwards (towards table). Ideal for picking.
        yaw = random.uniform(-3.14, 3.14)
        rpy = [3.1415 + random.uniform(-0.1, 0.1),
               random.uniform(-0.1, 0.1),
               yaw]

    return [x, y, z, rpy[0], rpy[1], rpy[2]]


def generate_scenarios():
    scenarios = []

    # Standard "Home" joints (Retracted)
    start_joints = [0.0, -0.5, 0.0, 0.0, 0.0, 0.0, 0.0]
    # Heuristic start position (End Effector approx location)
    start_pos_heuristic = [0.207, 0.0, 0.3]

    total_scenarios = 100

    # We will divide the 100 scenarios equally into 5 strategic categories
    # 20 Free Space, 20 Collinear, 20 Window/Wall, 20 Shelf/U-Shape, 20 Forest

    for i in range(total_scenarios):
        scenario = {
            "id": i,
            "start_joints": start_joints,
            "target_pose": [],
            "obstacles": [],
            "difficulty": "easy",
            "scenario_type": ""  # Added for easier evaluation tracking
        }

        # --- CATEGORY 1: FREE SPACE / SIMPLE AVOIDANCE (0-19) ---
        if i < 20:
            scenario["scenario_type"] = "free_space"
            scenario["difficulty"] = "easy"
            scenario["target_pose"] = get_smart_pose(orientation_type="downward")

            # 50% chance to put a small obstacle slightly off the direct path
            if random.random() > 0.5:
                tx, ty, tz = scenario["target_pose"][:3]
                mx = (start_pos_heuristic[0] + tx) / 2.0
                my = (start_pos_heuristic[1] + ty) / 2.0 + random.choice([-0.1, 0.1])
                mz = (start_pos_heuristic[2] + tz) / 2.0

                scenario["obstacles"].append({
                    "type": "sphere",
                    "pos": [mx, my, mz],
                    "radius": 0.05
                })

        # --- CATEGORY 2: STRICT COLLINEARITY (20-39) ---
        # A classic APF failure point. Obstacle is EXACTLY on the straight line to the target.
        elif i < 40:
            scenario["scenario_type"] = "collinear_trap"
            scenario["difficulty"] = "medium"
            scenario["target_pose"] = get_smart_pose(orientation_type="downward")

            tx, ty, tz = scenario["target_pose"][:3]

            # Place obstacle exactly at the 50% mark of the path vector
            mx = (start_pos_heuristic[0] + tx) / 2.0
            my = (start_pos_heuristic[1] + ty) / 2.0
            mz = (start_pos_heuristic[2] + tz) / 2.0

            scenario["obstacles"].append({
                "type": "sphere",
                "pos": [mx, my, mz],
                "radius": 0.07  # Large enough to fully block the direct vector
            })

        # --- CATEGORY 3: THE NARROW WINDOW / WALL (40-59) ---
        # Tests if the planner can slide along a wall to find an opening
        elif i < 60:
            scenario["scenario_type"] = "wall_window"
            scenario["difficulty"] = "hard"
            scenario["target_pose"] = get_smart_pose(orientation_type="downward")

            tx, ty, tz = scenario["target_pose"][:3]
            mx = (start_pos_heuristic[0] + tx) / 2.0
            my = (start_pos_heuristic[1] + ty) / 2.0
            mz = (start_pos_heuristic[2] + tz) / 2.0

            gap_width = 0.12  # Tight squeeze

            # Left Wall segment
            scenario["obstacles"].append({
                "type": "box",
                "pos": [mx, my - gap_width - 0.2, mz],
                "dim": [0.05, 0.4, 0.4]
            })
            # Right Wall segment
            scenario["obstacles"].append({
                "type": "box",
                "pos": [mx, my + gap_width + 0.2, mz],
                "dim": [0.05, 0.4, 0.4]
            })

        # --- CATEGORY 4: U-SHAPE / SHELF TRAP (60-79) ---
        # The ultimate local minima test. Planner must back out to escape.
        elif i < 80:
            scenario["scenario_type"] = "u_shape_shelf"
            scenario["difficulty"] = "hard"

            # CRITICAL FIX: Force horizontal orientation so the arm reaches straight forward
            scenario["target_pose"] = get_smart_pose(orientation_type="horizontal")
            tx, ty, tz = scenario["target_pose"][:3]

            # Build the shelf directly around the generated target pose
            # Assuming gripper approaches from -X direction

            # Shelf Floor (slightly below target)
            scenario["obstacles"].append({
                "type": "box",
                "pos": [tx + 0.05, ty, tz - 0.12],
                "dim": [0.3, 0.4, 0.02]
            })
            # Shelf Ceiling (slightly above target)
            scenario["obstacles"].append({
                "type": "box",
                "pos": [tx + 0.05, ty, tz + 0.12],
                "dim": [0.3, 0.4, 0.02]
            })
            # Shelf Back Wall (behind the target)
            scenario["obstacles"].append({
                "type": "box",
                "pos": [tx + 0.2, ty, tz],
                "dim": [0.02, 0.4, 0.26]
            })
            # Shelf Side Walls (Creates the strict U-shape trap)
            scenario["obstacles"].append({
                "type": "box",
                "pos": [tx + 0.05, ty - 0.2, tz],
                "dim": [0.3, 0.02, 0.26]
            })
            scenario["obstacles"].append({
                "type": "box",
                "pos": [tx + 0.05, ty + 0.2, tz],
                "dim": [0.3, 0.02, 0.26]
            })

        # --- CATEGORY 5: THE FOREST / CLUTTER (80-99) ---
        # Tests reaction to multiple repulsive fields interacting
        else:
            scenario["scenario_type"] = "forest_clutter"
            scenario["difficulty"] = "hard"
            scenario["target_pose"] = get_smart_pose(orientation_type="downward")

            tx, ty, tz = scenario["target_pose"][:3]
            num_trees = random.randint(4, 6)

            for _ in range(num_trees):
                # Randomize obstacles inside the workspace area
                ox = random.uniform(0.25, 0.5)
                oy = random.uniform(-0.4, 0.4)
                oz = random.uniform(0.1, 0.4)

                # Ensure obstacles don't spawn exactly ON the start or target poses
                if distance([ox, oy, oz], [tx, ty, tz]) > 0.12 and \
                        distance([ox, oy, oz], start_pos_heuristic) > 0.15:
                    scenario["obstacles"].append({
                        "type": "cylinder",
                        "pos": [ox, oy, oz],
                        "radius": random.uniform(0.02, 0.05),
                        "height": 0.5
                    })

        scenarios.append(scenario)

    with open("scenarios.json", "w") as f:
        json.dump(scenarios, f, indent=4)
    print(f"Generated {total_scenarios} highly structured scenarios in scenarios.json")


if __name__ == "__main__":
    generate_scenarios()