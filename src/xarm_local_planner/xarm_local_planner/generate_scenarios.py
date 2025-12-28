import json
import random
import math


def get_random_pose(min_r=0.3, max_r=0.6):
    """Generates a reachable target pose in front of the robot"""
    r = random.uniform(min_r, max_r)
    theta = random.uniform(-1.0, 1.0)  # +/- ~60 degrees
    z = random.uniform(0.1, 0.5)

    x = r * math.cos(theta)
    y = r * math.sin(theta)

    # Standard downward facing gripper orientation
    return [x, y, z, 3.14, 0.0, 0.0]


def generate_scenarios():
    scenarios = []

    # Standard "Home" joints
    start_joints = [0.0, -0.5, 0.0, 0.0, 0.0, 0.0, 0.0]

    for i in range(100):
        scenario = {
            "id": i,
            "start_joints": start_joints,
            "target_pose": get_random_pose(),
            "obstacles": [],
            "difficulty": "easy"
        }

        # --- LEVEL 1: EASY (Free Space / Distant Obstacles) ---
        if i < 30:
            scenario["difficulty"] = "easy"
            # Just a random box far away
            scenario["obstacles"].append({
                "type": "box",
                "pos": [0.8, 0.8, 0.0],  # Far corner
                "dim": [0.1, 0.1, 0.1]
            })

        # --- LEVEL 2: MEDIUM (Simple Blocking) ---
        elif i < 70:
            scenario["difficulty"] = "medium"
            tx, ty, tz = scenario["target_pose"][:3]

            # Place obstacle exactly halfway between Home and Target
            # Home TCP is approx [0.2, 0, 0.2]
            ox = (tx + 0.2) / 2.0
            oy = ty / 2.0
            oz = (tz + 0.2) / 2.0

            # 50% chance of Sphere vs Box
            if random.random() < 0.5:
                scenario["obstacles"].append({
                    "type": "sphere",
                    "pos": [ox, oy, oz],
                    "radius": 0.08
                })
            else:
                scenario["obstacles"].append({
                    "type": "box",
                    "pos": [ox, oy, oz],
                    "dim": [0.05, 0.2, 0.2]  # A wide wall
                })

        # --- LEVEL 3: HARD (Local Minima Traps) ---
        else:
            scenario["difficulty"] = "hard"
            tx, ty, tz = scenario["target_pose"][:3]

            # Create a "U" shape or "Table" that requires backing up or going around
            # We place a "Floor" under the target and a "Ceiling" above it

            # Floor plate
            scenario["obstacles"].append({
                "type": "box",
                "pos": [tx, ty, tz - 0.15],
                "dim": [0.15, 0.15, 0.02]
            })
            # Ceiling plate
            scenario["obstacles"].append({
                "type": "box",
                "pos": [tx, ty, tz + 0.15],
                "dim": [0.15, 0.15, 0.02]
            })
            # Back Wall (creating a C-shape)
            # Offset slightly further back in X
            scenario["obstacles"].append({
                "type": "box",
                "pos": [tx + 0.15, ty, tz],
                "dim": [0.02, 0.15, 0.15]
            })

        scenarios.append(scenario)

    with open("scenarios.json", "w") as f:
        json.dump(scenarios, f, indent=4)
    print("Generated 100 scenarios in scenarios.json")


if __name__ == "__main__":
    generate_scenarios()