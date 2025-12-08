#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from rclpy.publisher import Publisher

import pdb

# Integration deiner Module
from mam_eurobot_2026.arm_kinematics.trajectory_generators import quintic_trajectory
from mam_eurobot_2026.arm_kinematics.kinematics_5dof import fk, J_task_num, ik_least_squares, compute_best_q_for_goal


def quaternion_to_yaw_roll(q):
    """
    Hilfsfunktion: extrahiere yaw und roll aus Quaternion.
    Achtung: hier nur einfache Annäherung, für präzise Euler-Konvention ggf. anpassen.
    """
    qw, qx, qy, qz = q.w, q.x, q.y, q.z

    # Yaw (Z-Achse)
    yaw = np.arctan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz))
    # Roll (X-Achse)
    roll = np.arctan2(2.0*(qw*qx + qy*qz), 1.0 - 2.0*(qx*qx + qy*qy))

    return yaw, roll


class ArmCommander(Node):
    def __init__(self,
                 joint_names=["joint_1","joint_2","joint_3","joint_4","joint_5"],
                 sense=None):
        super().__init__("gazebo_arm_commander")
        self.joint_names = joint_names
        self.dof = len(joint_names)
        self.sense = np.ones(self.dof) if sense is None else np.asarray(sense)

        # Publisher: ein Float64 pro Joint für JointPositionController
        self.pubs: list[Publisher] = []
        for j in self.joint_names:
            topic = f"/{j}/position_cmd"
            self.pubs.append(self.create_publisher(Float64, topic, 10))

        # Subscriber für joint_states
        self._latest_positions = np.zeros(self.dof)
        self.create_subscription(JointState, "/joint_states", self._joint_states_cb, 10)

        # Subscriber für /grab_nut Pose
        self.create_subscription(Pose, "/grab_nut", self._grab_nut_cb, 10)

        self.get_logger().info("ArmCommander ready (JointPositionController mode)")

    def _joint_states_cb(self, msg: JointState):
        joint_state_dict = {j: p for j, p in zip(msg.name, msg.position)}
        self._latest_positions = np.array([joint_state_dict.get(n, 0.0) for n in self.joint_names])
        # self._latest_positions = [0.0, 1.57, 0.0, 0.0, 0.0]

    def get_current_joint_states(self):
        return self._latest_positions * self.sense

    def send_joint_positions(self, positions):
        """
        Sende eine Gelenkkonfiguration direkt an die JointPositionController
        """
        positions = np.asarray(positions) * self.sense
        for i in range(self.dof):
            msg = Float64()
            msg.data = float(positions[i])
            self.pubs[i].publish(msg)

    def follow_joint_trajectory(self, joint_traj: np.ndarray, t_vec: np.ndarray) -> None:
        """
        Fahre eine komplette Trajektorie ab, indem direkt Float64-Kommandos gesendet werden
        """
        self.get_logger().info("Following trajectory with JointPositionControllers...")
        for j_i, t_i in zip(joint_traj, t_vec):
            self.send_joint_positions(j_i)
            rclpy.spin_once(self, timeout_sec=float(t_i))
            self.get_logger().info("Followed joint position")
        self.get_logger().debug("Done moving")

    def follow_quintic_to(self, q_target, t_f=2.0, n=200):
        q0 = self.get_current_joint_states()
        t_vec, q_traj = quintic_trajectory(q0, q_target, t_f=t_f, n=n)
        self.follow_joint_trajectory(q_traj, t_vec)

    # --- Kinematics Integration ---
    def fk_pose(self, q_vec):
        return fk(q_vec)

    def jacobian_rank(self, q_vec):
        Jn = J_task_num(q_vec)
        return np.linalg.matrix_rank(Jn)

    def ik_to_pose(self, pose, q_init=None):
        if q_init is None:
            self.get_logger().warning(f"Not moving because q_init is None")
            q_init = self.get_current_joint_states()
        return ik_least_squares(pose, q_init)

    # --- Callback für /grab_nut ---
    def _grab_nut_cb(self, msg: Pose):
        # Pose extrahieren
        x = msg.position.x * 1e3
        y = msg.position.y * 1e3
        z = msg.position.z * 1e3
        yaw, roll = quaternion_to_yaw_roll(msg.orientation)

        goal_pose = [x, y, z, roll, yaw]
        self.get_logger().info(f"Received nut pose: {goal_pose}")

        # pdb.set_trace()
        # IK berechnen
        q_goal = self.ik_to_pose(goal_pose, q_init=self.get_current_joint_states())
        # q_goal = compute_best_q_for_goal(goal=goal_pose)

        self.get_logger().info(f"IK solution for nut: {q_goal}")
        self.get_logger().info(f"FK(goal check): {fk(q_goal)}")


        self.send_joint_positions(q_goal)
        # # Quintic Trajektorie fahren
        # self.follow_quintic_to(q_goal, t_f=3.0, n=5)

        self.get_logger().info(f"Done")


def main():
    rclpy.init()
    commander = ArmCommander()
    try:
        rclpy.spin(commander)
    except KeyboardInterrupt:
        pass
    commander.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
