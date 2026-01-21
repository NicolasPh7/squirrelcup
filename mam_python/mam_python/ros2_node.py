#!/usr/bin/env python3
"""
ROS2 Node Wrapper for MAM Python Robot
Path planning OK, path following FIXED
"""

import math
import threading
import time
from typing import List

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Path, OccupancyGrid
from std_msgs.msg import String

# Local modules
from mam_python.decision import StrategyEngine, MissionManager, ActionPlanner, ActionType
from mam_python.planning import PathPlanner, CollisionChecker, TrajectoryGenerator
from mam_python.control import MotionController, ArmController
from mam_python.perception import MapBuilder, ObjectManager, TargetSelector, Target
from mam_python.scoring import ScoreTracker, ScoreOptimizer, ScorePredictor
from mam_python.core import RobotState, GameState


class MAMRobotNode(Node):

    def __init__(self):
        super().__init__('mam_robot_node')
        self.logger = self.get_logger()

        # --- States ---
        self.robot_state = RobotState()
        self.game_state = GameState()

        # --- Strategy ---
        self.strategy_engine = StrategyEngine()
        self.mission_manager = MissionManager()
        self.action_planner = ActionPlanner()
        for a in [ActionType.MOVE_TO, ActionType.GRAB, ActionType.RETURN, ActionType.RELEASE, ActionType.WAIT]:
            self.action_planner.add_action(a)

        # --- Planning ---
        self.path_planner = PathPlanner()
        self.map_builder = MapBuilder(resolution=0.1)
        self.target_selector = TargetSelector()

        # --- Control ---
        self.motion_controller = MotionController()
        self.arm_controller = ArmController()

        # --- Targets ---
        self.targets: List[Target] = []
        self.current_target: Target | None = None

        self.home_position = Target(id=0, x=2.8, y=1.8)
        self.robot_state.update_position(self.home_position.x, self.home_position.y, -math.pi / 2)

        self.aruco_map_markers = [
            Target(id=20, x=0.6, y=1.4),
            Target(id=21, x=2.4, y=1.4),
            Target(id=22, x=0.6, y=0.6),
            Target(id=23, x=2.4, y=0.6),
        ]

        # --- Scoring (unused for now) ---
        self.score_tracker = ScoreTracker()
        self.score_optimizer = ScoreOptimizer()
        self.score_predictor = ScorePredictor()

        # --- ROS interfaces ---
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.grid_pub = self.create_publisher(OccupancyGrid, '/occupancy_grid', 10)

        self.object_detector_sub = self.create_subscription(
            MarkerArray, '/supposed_objects', self.object_detector_callback, 10
        )
        self.odom_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/corrected_pose', self.odom_callback, 10
        )
        self.nuts_sub = self.create_subscription(
            MarkerArray, '/nuts', self.nuts_callback, 10
        )

        # --- Path following ---
        self.current_path: list[tuple[float, float]] | None = None
        self.lookahead_dist = 0.25
        self.max_linear_speed = 0.6
        self.max_angular_speed = 2.0
        self.k_angular = 2.5

        self.tick_period = 0.2
        self.start_control_loop()

        self.logger.info("MAM Robot Node started")

    # ---------------------------------------------------------
    # Callbacks
    # ---------------------------------------------------------

    def object_detector_callback(self, msg: MarkerArray):
        if self.map_builder.update_from_marker_array(msg, self.robot_state.get_position()):
            for m in self.aruco_map_markers:
                self.map_builder.update_cell(x=m.x, y=m.y, size=0.1)
            self.map_builder.publish_occupancy_grid(
                self.get_clock().now().to_msg(), self.grid_pub
            )

    def odom_callback(self, msg: PoseWithCovarianceStamped):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation

        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        theta = math.atan2(siny, cosy)

        self.robot_state.update_position(x, y, theta)

    def nuts_callback(self, msg: MarkerArray):
        self.targets = [
            Target(
                id=m.id,
                x=m.pose.position.x,
                y=m.pose.position.y,
                priority=0,
                score=0
            )
            for m in msg.markers if m.action != m.DELETEALL
        ]

    # ---------------------------------------------------------
    # Path Planning
    # ---------------------------------------------------------

    def prepare_movement(self, targets: list[Target]):
        pos = self.robot_state.get_position()
        target = self.target_selector.distance_priority(targets, pos.x, pos.y)

        if target is None:
            return None

        self.current_target = target

        raw_path = self.path_planner.plan_A_Star(
            start=(pos.x, pos.y),
            goal=(target.x, target.y),
            map=self.map_builder
        )

        if raw_path is None:
            return None

        smooth = self.path_planner.smooth_with_catmull_rom(
            raw_path, samples_per_seg=8, kappa_max=0.3
        )

        return smooth

    # ---------------------------------------------------------
    # Path Following (PURE PURSUIT SIMPLE)
    # ---------------------------------------------------------

    def follow_path(self):
        if self.current_path is None or len(self.current_path) < 2:
            return

        pos = self.robot_state.get_position()
        rx, ry, rtheta = pos.x, pos.y, pos.theta

        dists = [math.hypot(px - rx, py - ry) for px, py in self.current_path]
        closest = dists.index(min(dists))
        lookahead = min(closest + 1, len(self.current_path) - 1)

        tx, ty = self.current_path[lookahead]
        dx = tx - rx
        dy = ty - ry

        distance = math.hypot(dx, dy)
        target_angle = math.atan2(dy, dx)
        angle_error = self.normalize_angle(target_angle - rtheta)

        cmd = Twist()

        if abs(angle_error) > 0.4:
            cmd.linear.x = 0.0
            cmd.angular.z = self.k_angular * angle_error
        else:
            cmd.linear.x = min(self.max_linear_speed, distance)
            cmd.angular.z = self.k_angular * angle_error

        cmd.angular.z = max(
            -self.max_angular_speed,
            min(self.max_angular_speed, cmd.angular.z)
        )

        self.cmd_vel_pub.publish(cmd)

        if distance < 0.15 and lookahead == len(self.current_path) - 1:
            self.stop_robot()
            self.current_path = None
            self.logger.info("Path completed")

    def stop_robot(self):
        self.cmd_vel_pub.publish(Twist())

    def normalize_angle(self, a):
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
        return a

    # ---------------------------------------------------------
    # Control Loop
    # ---------------------------------------------------------

    def start_control_loop(self):
        def loop():
            while rclpy.ok():
                self.control_loop()
                time.sleep(self.tick_period)
        threading.Thread(target=loop, daemon=True).start()

    def control_loop(self):
        pos = self.robot_state.get_position()

        strategy = self.strategy_engine.decide_next_action(
            robot_position=(pos.x, pos.y),
            nearby_targets=self.targets,
            battery_level=100
        )

        new_path = None

        if strategy.action == "return_home":
            new_path = self.prepare_movement([self.home_position])
        elif strategy.action == "grab_nearest":
            new_path = self.prepare_movement(self.targets)
        elif strategy.action == "explore":
            new_path = self.prepare_movement(self.aruco_map_markers)

        if new_path is not None:
            self.current_path = new_path
            self.publish_path(new_path)

        if self.current_path is not None:
            self.follow_path()

    # ---------------------------------------------------------

    def publish_path(self, path_points):
        msg = Path()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()

        for x, y in path_points:
            p = PoseStamped()
            p.header.frame_id = "map"
            p.pose.position.x = x
            p.pose.position.y = y
            p.pose.orientation.w = 1.0
            msg.poses.append(p)

        self.path_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MAMRobotNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()


if __name__ == '__main__':
    main()
