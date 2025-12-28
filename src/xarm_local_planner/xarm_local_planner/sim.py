#!/usr/bin/env python3
"""
xArm Planning Scene (Calculation Engine)
- SYNC: Subscribes to /xarm/joint_states to mirror the Real/Twin robot.
- MAP: Loads obstacles from scenarios.json.
- QUERIES: Provides services for 'Check Collision' and 'Get APF Distances'.
"""

import rclpy
from rclpy.node import Node
import pybullet as p
import pybullet_data
import json
import numpy as np
from threading import Lock
from sensor_msgs.msg import JointState
from xarm_msgs.srv import MoveJoint, GetFloat32List


# We will use GetFloat32List to return [dist, dx, dy, dz] for APF

class XArmPlanningScene(Node):
    def __init__(self):
        super().__init__('xarm_planning_scene')

        # ===== CONFIG =====
        self.declare_parameter('urdf_path', '/home/ros2_ws/plan_move_execute/src/assets/models/xarm7.urdf')
        self.urdf_path = self.get_parameter('urdf_path').value

        # Load Scenarios
        try:
            with open("scenarios.json", "r") as f:
                self.scenarios = json.load(f)
        except:
            self.scenarios = []
            self.get_logger().warn("scenarios.json not found!")

        # ===== PYBULLET (HEADLESS) =====
        # We use DIRECT mode (no GUI) because this is just a calculation engine
        self.client = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        # Load Static World
        self.plane_id = p.loadURDF("plane.urdf")
        self.robot_id = p.loadURDF(self.urdf_path, [0, 0, 0], useFixedBase=True)

        # Internal State
        self.sim_lock = Lock()
        self.obstacle_ids = []
        self.current_joints = [0.0] * 7

        # Map Joints
        self.arm_indices = []
        for i in range(p.getNumJoints(self.robot_id)):
            if "joint" in p.getJointInfo(self.robot_id, i)[1].decode('utf-8'):
                self.arm_indices.append(i)
        self.arm_indices = self.arm_indices[:7]

        # ===== ROS INTERFACE =====

        # 1. INPUT: Sync with Real Robot
        self.create_subscription(JointState, '/xarm/joint_states', self._sync_callback, 10)

        # 2. INPUT: Load Scenario Command (Using MoveJoint just for the 'data' field logic or custom srv)
        # We will assume a service call handles this or parameter.
        # For this example, I'll add a helper service to switch levels.
        self.create_service(MoveJoint, '/planning/load_scenario', self._srv_load_scenario)

        # 3. QUERY: Check Collision (Hypothetical)
        self.create_service(MoveJoint, '/planning/check_collision', self._srv_check_collision)

        # 4. QUERY: Get Closest Points (For APF)
        # Returns [link_idx, distance, dir_x, dir_y, dir_z, ...] flat list
        self.create_service(GetFloat32List, '/planning/get_apf_distances', self._srv_get_apf_data)

        # Load default
        self.load_scenario(0)
        self.get_logger().info("Planning Engine Ready (Headless). Syncing with /xarm/joint_states")

    # ==================== CORE LOGIC ====================

    def load_scenario(self, scenario_id):
        if scenario_id >= len(self.scenarios): return
        data = self.scenarios[scenario_id]

        with self.sim_lock:
            # Clear old
            for uid in self.obstacle_ids: p.removeBody(uid)
            self.obstacle_ids = []

            # Spawn new
            for obs in data['obstacles']:
                pos = obs['pos']
                if obs['type'] == 'box':
                    h = [d / 2 for d in obs['dim']]
                    col = p.createCollisionShape(p.GEOM_BOX, halfExtents=h)
                else:
                    col = p.createCollisionShape(p.GEOM_SPHERE, radius=obs['radius'])

                # Create Body (Static)
                uid = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=col, basePosition=pos)
                self.obstacle_ids.append(uid)

            # Note: We do NOT reset the robot here. The robot must stay synced to the real topic.
            self.get_logger().info(f"Loaded Scenario {scenario_id}: {len(self.obstacle_ids)} obstacles")

    def _sync_callback(self, msg):
        """
        Real-time Twin Sync.
        When Real Robot moves, this Ghost Robot snaps to match it instantly.
        """
        if len(msg.position) >= 7:
            with self.sim_lock:
                self.current_joints = msg.position[:7]
                for i, angle in enumerate(self.current_joints):
                    p.resetJointState(self.robot_id, self.arm_indices[i]+1, angle)

    # ==================== SERVICES ====================

    def _srv_load_scenario(self, req, res):
        """API to change level from the Planner"""
        # We abuse the 'angles[0]' field to pass the ID, assuming int
        s_id = int(req.angles[0])
        self.load_scenario(s_id)
        res.ret = 0
        res.message = f"Loaded Scenario {s_id}"
        return res

    def _srv_check_collision(self, req, res):
        """
        Hypothetical Check: "If I move to these angles, is it safe?"
        Teleports ghost -> Checks -> Teleports back to synced state.
        """
        target = req.angles[:7]
        is_hit = False

        with self.sim_lock:
            # 1. Save Sync State (already in self.current_joints)

            # 2. Teleport to Hypothesis
            for i, val in enumerate(target):
                p.resetJointState(self.robot_id, self.arm_indices[i], val)

            # 3. Check
            p.performCollisionDetection()
            contacts = p.getContactPoints(self.robot_id)
            for c in contacts:
                # Ignore base-floor contact
                if c[2] == self.plane_id and c[3] <= 1: continue
                is_hit = True
                break

            # 4. Restore Sync State (Snap back to real robot)
            for i, val in enumerate(self.current_joints):
                p.resetJointState(self.robot_id, self.arm_indices[i], val)

        res.ret = 1 if is_hit else 0
        res.message = "Collision" if is_hit else "Safe"
        return res

    def _srv_get_apf_data(self, req, res):
        """
        DEBUG VERSION: Prints details to terminal
        """
        # 1. Verify what we are checking
        critical_links = [2, 4, 6]
        data = []

        self.get_logger().info(f"--- Checking APF (Robot ID: {self.robot_id}) ---")

        with self.sim_lock:
            # Check how many bodies exist in the world
            num_bodies = p.getNumBodies()
            self.get_logger().info(f"Total Physics Bodies: {num_bodies}")

            for link_idx in critical_links:
                # distance=10.0 is HUGE to ensure we see SOMETHING
                results = p.getClosestPoints(bodyA=self.robot_id, bodyB=-1, distance=10.0, linkIndexA=link_idx)

                # Debug print
                if not results:
                    self.get_logger().warn(f"Link {link_idx}: No contacts found!")

                min_dist = 100.0
                best_vec = [0.0, 0.0, 0.0]
                found = False

                for c in results:
                    obs_id = c[2]
                    dist = c[8]

                    # Ignore Robot Self-Collision
                    if obs_id == self.robot_id:
                        continue

                        # OPTIONAL: Ignore Floor (Plane) if you only want Object Distances
                    # if obs_id == self.plane_id: continue

                    if dist < min_dist:
                        min_dist = dist
                        found = True
                        p_robot = np.array(c[5])
                        p_obs = np.array(c[6])
                        vec = p_obs - p_robot
                        best_vec = vec.tolist()

                if found:
                    self.get_logger().info(f"Link {link_idx} -> Obs {obs_id}: Dist={min_dist:.3f}m")
                    data.extend([float(link_idx), float(min_dist), best_vec[0], best_vec[1], best_vec[2]])
                else:
                    self.get_logger().info(f"Link {link_idx}: Only self-collisions found.")

        res.ret = 0
        res.datas = data
        return res


def main():
    rclpy.init()
    node = XArmPlanningScene()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()