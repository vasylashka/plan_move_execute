import rclpy
from rclpy.node import Node
import pybullet as p
import pybullet_data
import json
import numpy as np
from threading import Lock
from sensor_msgs.msg import JointState
from xarm_local_msgs.srv import GetApfDistances, LoadScenario, CheckCollision
from xarm_local_msgs.msg import ApfLinkData



class XArmPlanningScene(Node):
    def __init__(self):
        super().__init__('xarm_planning_scene')

        self.declare_parameter('urdf_path', '/home/ros2_ws/plan_move_execute/src/assets/models/xarm7.urdf')
        self.urdf_path = self.get_parameter('urdf_path').value

        try:
            with open("scenarios.json", "r") as f:
                self.scenarios = json.load(f)
        except:
            self.scenarios = []
            self.get_logger().warn("scenarios.json not found!")

        self.client = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        self.plane_id = p.loadURDF("plane.urdf")
        self.robot_id = p.loadURDF(self.urdf_path, [0, 0, 0], useFixedBase=True)

        self.sim_lock = Lock()
        self.obstacle_ids = []
        self.current_joints = [0.0] * 7

        self.arm_indices = []
        for i in range(p.getNumJoints(self.robot_id)):
            if "joint" in p.getJointInfo(self.robot_id, i)[1].decode('utf-8'):
                self.arm_indices.append(i)
        self.arm_indices = self.arm_indices[:7]

        # ===== ROS INTERFACE =====

        self.create_subscription(JointState, '/xarm/joint_states', self._sync_callback, 10)

        self.create_service(LoadScenario, '/planning/load_scenario', self._srv_load_scenario)

        self.create_service(CheckCollision, '/planning/check_collision', self._srv_check_collision)

        self.create_service(GetApfDistances, '/planning/get_apf_distances', self._srv_get_apf_data)

        self.load_scenario(0)
        self.get_logger().info("Planning Engine Ready (Headless). Syncing with /xarm/joint_states")

    # ==================== CORE LOGIC ====================

    def load_scenario(self, scenario_id):
        if scenario_id >= len(self.scenarios): return
        data = self.scenarios[scenario_id]

        with self.sim_lock:
            for uid in self.obstacle_ids: p.removeBody(uid)
            self.obstacle_ids = []

            if hasattr(self, 'target_id') and self.target_id is not None:
                p.removeBody(self.target_id)
                self.target_id = None

            for obs in data['obstacles']:
                pos = obs['pos']
                if obs['type'] == 'box':
                    h = [d / 2 for d in obs['dim']]
                    col = p.createCollisionShape(p.GEOM_BOX, halfExtents=h)
                    vis = p.createVisualShape(p.GEOM_BOX, halfExtents=h, rgbaColor=[0.5, 0.5, 0.5, 1])
                else:
                    col = p.createCollisionShape(p.GEOM_SPHERE, radius=obs['radius'])
                    vis = p.createVisualShape(p.GEOM_SPHERE, radius=obs['radius'], rgbaColor=[0.5, 0.5, 0.5, 1])

                uid = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=col, baseVisualShapeIndex=vis,
                                        basePosition=pos)
                self.obstacle_ids.append(uid)

            if 'target_pose' in data:
                t_pos = data['target_pose'][:3]

                t_vis = p.createVisualShape(p.GEOM_SPHERE, radius=0.03, rgbaColor=[0, 1, 0, 0.8])

                self.target_id = p.createMultiBody(baseMass=0, baseVisualShapeIndex=t_vis, basePosition=t_pos)

            self.get_logger().info(f"Loaded Scenario {scenario_id}: {len(self.obstacle_ids)} obstacles + Target")

    def _sync_callback(self, msg):
        if len(msg.position) >= 7:
            with self.sim_lock:
                self.current_joints = msg.position[:7]
                for i, angle in enumerate(self.current_joints):
                    p.resetJointState(self.robot_id, self.arm_indices[i]+1, angle)

    # ==================== SERVICES ====================

    def _srv_load_scenario(self, req, res):
        """API to change level from the Planner"""
        # Clean and type-safe!
        s_id = req.scenario_index

        if s_id < 0 or s_id >= len(self.scenarios):
            res.success = False
            res.message = f"Invalid Scenario ID: {s_id}"
            return res

        self.load_scenario(s_id)
        res.success = True
        res.message = f"Loaded Scenario {s_id}"
        return res

    def _srv_check_collision(self, req, res):
        """
        Hypothetical Check: "If I move to these angles, is it safe?"
        """
        target = req.joint_positions

        if len(target) < 7:
            res.collision_detected = False
            res.message = "Error: Need at least 7 joint angles"
            return res

        is_hit = False

        with self.sim_lock:
            for i in range(7):
                p.resetJointState(self.robot_id, self.arm_indices[i], target[i])

            p.performCollisionDetection()
            contacts = p.getContactPoints(self.robot_id)
            for c in contacts:
                if c[2] == self.plane_id and c[3] <= 1: continue
                is_hit = True
                break

            for i, val in enumerate(self.current_joints):
                p.resetJointState(self.robot_id, self.arm_indices[i], val)

        res.collision_detected = is_hit
        res.message = "Collision" if is_hit else "Safe"
        return res

    def _srv_get_apf_data(self, req, res):
        critical_links = [2, 4, 6, 9, 11, 14]

        link_names = {
            2: "Shoulder", 4: "Elbow", 6: "Wrist",
            9: "Hand Base", 11: "Left Finger", 14: "Right Finger"
        }

        targets = [self.plane_id] + self.obstacle_ids

        with self.sim_lock:
            for link_idx in critical_links:
                min_dist = 100.0
                best_vec = [0.0, 0.0, 0.0]
                found = False

                for target_id in targets:
                    if target_id == self.robot_id: continue
                    results = p.getClosestPoints(bodyA=self.robot_id, bodyB=target_id, distance=10.0,
                                                 linkIndexA=link_idx)
                    if not results: continue

                    for c in results:
                        dist = c[8]
                        if dist < min_dist:
                            min_dist = dist
                            found = True
                            p_robot = np.array(c[5])
                            p_obs = np.array(c[6])
                            best_vec = (p_obs - p_robot).tolist()

                if found:
                    msg = ApfLinkData()
                    msg.link_index = int(link_idx)
                    msg.link_name = link_names.get(link_idx, f"Link_{link_idx}")
                    msg.min_distance = float(min_dist)
                    # Assign Vector3
                    msg.direction_vector.x = float(best_vec[0])
                    msg.direction_vector.y = float(best_vec[1])
                    msg.direction_vector.z = float(best_vec[2])

                    # Add to response list
                    res.data.append(msg)

        return res


def main():
    rclpy.init()
    node = XArmPlanningScene()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()