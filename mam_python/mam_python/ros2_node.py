#!/usr/bin/env python3

"""
ROS2 Node Wrapper for MAM Python Robot
Integrates all robot modules with ROS2 Humble and Gazebo
"""
import pdb

import math
import threading
import time

from typing import List

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from visualization_msgs.msg import MarkerArray 
from sensor_msgs.msg import LaserScan, JointState, Image
from nav_msgs.msg import Path, OccupancyGrid
from std_msgs.msg import String, Int32
from builtin_interfaces.msg import Time

# Import local modules
from mam_python.decision import StrategyEngine, MissionManager, ActionPlanner, ActionType
from mam_python.planning import PathPlanner, CollisionChecker, TrajectoryGenerator
from mam_python.control import MotionController, ArmController
from mam_python.perception import MapBuilder, ObjectManager, TargetSelector, Target
from mam_python.scoring import ScoreTracker, ScoreOptimizer, ScorePredictor
from mam_python.core import RobotState, GameState


class MAMRobotNode(Node):
    """Main ROS2 Node for MAM Robot"""
    
    def __init__(self):
        super().__init__('mam_robot_node')
        self.logger = self.get_logger()  # FIX: Use ROS2 native get_logger()
        
        # Initialize all robot modules
        self.robot_state = RobotState()
        self.game_state = GameState()
        
        self.strategy_engine = StrategyEngine()
        self.mission_manager = MissionManager()
        self.action_planner = ActionPlanner()
        self.action_planner.add_action(ActionType.MOVE_TO)
        self.action_planner.add_action(ActionType.GRAB)
        self.action_planner.add_action(ActionType.RETURN)
        self.action_planner.add_action(ActionType.RELEASE)
        self.action_planner.add_action(ActionType.WAIT)

        self.path_planner = PathPlanner()
        self.collision_checker = CollisionChecker()
        self.trajectory_generator = TrajectoryGenerator()
        self.path : list[tuple[float, float]]  | None = None

        self.motion_controller = MotionController()
        self.arm_controller = ArmController()
        
        self.map_builder = MapBuilder(resolution=0.01)
        self.object_manager = ObjectManager()
        self.target_selector = TargetSelector()
        self.home_position = Target(id=0, x=2.8, y=1.8) #see https://www.eurobot.org/wp-content/uploads/2025/09/Eurobot_General_Rules_1.2_EN.pdf
        self.robot_state.update_position(self.home_position.x, self.home_position.y, -1.57079633)
        self.targets: List[Target] = []
        self.current_target: Target | None = None
        self.aruco_map_markers = [
            Target(id=20, x=0.6, y=1.4),
            Target(id=21, x=2.4, y=1.4),
            Target(id=22, x=0.6, y=0.6),
            Target(id=23, x=2.4, y=0.6),
        ]

        self.score_tracker = ScoreTracker()
        self.score_optimizer = ScoreOptimizer()
        self.score_predictor = ScorePredictor()

        self.min_lookahead_dist = 0.05
        self.max_linear_speed = 0.6
        self.max_angular_speed = 10.0
        self.k_angular = 2
        self.k_linear = 15.0
        self.tick_period = 0.02
        
        # Create publishers
        # self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_intent', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.grid_pub = self.create_publisher(OccupancyGrid, "/occupancy_grid", 10)
        # self.arm_cmd_pub = self.create_publisher(JointState, '/robot/arm_commands', 10)
        # self.score_pub = self.create_publisher(Int32, '/robot/score', 10)
        # self.status_pub = self.create_publisher(String, '/robot/status', 10)
        
        # Create subscribers
        self.object_detector_sub = self.create_subscription(MarkerArray, '/supposed_objects', self.object_detector_callback, 10)
        self.odom_sub = self.create_subscription(PoseWithCovarianceStamped, '/corrected_pose', self.odom_callback, 10)
        self.nuts_sub = self.create_subscription(MarkerArray,"/nuts",self.nuts_callback,10)
        # self.camera_sub = self.create_subscription(Image, '/camera/image', self.camera_callback, 10)
        
        # self.timer = self.create_timer(self.tick_period, self.control_loop)
        self.trajectory_follower_thread : threading.Thread | None = None
        self.control_loop_thread : threading.Thread | None = None
        self.exploring = False

        self.logger.info("MAM Robot Node initialized!")

        self.start_control_loop()
    
    def publish_path(self, path_points: list[tuple[float, float]]):
        """
        Publiziert einen geplanten Pfad als nav_msgs/Path.
        
        Args:
            path_points: Liste von (x, y) Koordinaten in Weltkoordinaten (Meter)
        """
        path_msg = Path()
        poses = []
        path_msg.header.frame_id = "map"   # oder dein globales Frame
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for (x, y) in path_points:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0  # keine Rotation
            poses.append(pose)

        path_msg.poses = poses
        self.path_pub.publish(path_msg)
        self.get_logger().info(f"Published path with {len(path_points)} points")


    def object_detector_callback(self, msg: MarkerArray):
        """Process LIDAR data"""
        try:
            # pdb.set_trace()
            if self.map_builder.update_from_marker_array(msg, self.robot_state.get_position(), oversize=0.05):
                for m in self.aruco_map_markers:
                    self.map_builder.update_cell(x=m.x, y=m.y, size=0.005) # Add Arcuco Markers as obstacle

                self.map_builder.publish_occupancy_grid(self.get_clock().now().to_msg(), self.grid_pub)
        except Exception as e:
            self.logger.error(f"LIDAR error: {e}")
    
    # def camera_callback(self, msg: Image):
    #     """Process camera data"""
    #     try:
    #         objects = self.object_manager.detect_objects(msg)
    #         self.target_selector.update_targets(objects, self.robot_state.position)
    #     except Exception as e:
    #         self.logger.error(f"Camera error: {e}")
    
    def odom_callback(self, msg: PoseWithCovarianceStamped):
        """Update odometry"""
        try:
            # Position
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y

            # Orientation (Quaternion → Yaw)
            q = msg.pose.pose.orientation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            theta = math.atan2(siny_cosp, cosy_cosp)

            # Update robot state
            self.robot_state.update_position(x, y, theta)
            # self.get_logger().info(f"Updated robot position {self.robot_state.get_position()}")

        except Exception as e:
            self.logger.error(f"Odom error: {e}")

    def nuts_callback(self, msg: MarkerArray):
        """Update targets list from incoming MarkerArray."""
        updated_targets: List[Target] = []

        for marker in msg.markers:
            if marker.action == marker.DELETEALL: 
                continue
            
            # Extrahiere Basisdaten
            tid = marker.id
            x = marker.pose.position.x
            y = marker.pose.position.y

            priority = 0
            score = 0

            updated_targets.append(Target(
                id=tid,
                x=x,
                y=y,
                priority=priority,
                score=score
            ))

        if len(updated_targets) == 0:
            return
        # Ersetze die alte Liste durch die neue
        self.targets = updated_targets
        # self.get_logger().info(f"Targets updated: {len(self.targets)} items")

    
    def prepare_movement(self, targets: list[Target]) -> list[tuple[float, float]] | None:
        """
        Selects a target based on distance priority, updates current target,
        checks if reached, and plans a path using A*.
        """

        actual_pos = self.robot_state.get_position()
        target = self.target_selector.distance_priority(targets, actual_pos.x, actual_pos.y)

        # --- Target selection / update ---
        if self.current_target is None:
            if target is None:
                self.get_logger().info("prepare_movement: No available target")
                return None
            self.current_target = target
        elif target:
            self.get_logger().info(
                    f"prepare_movement: Updating current target coordinates"
                )
            self.current_target = target

        if not self.current_target:
            return None

        self.get_logger().info(f"prepare_movement: Current target {self.current_target}")

        # --- Check if target reached ---
        if self.target_reached(self.current_target):
            self.stop_robot()
            self.action_planner.complete_current_action()
            self.current_target = None
            self.get_logger().info("prepare_movement: Target reached")
            return None

        # --- Path planning ---
        self.get_logger().info(f"prepare_movement: Planning path to {self.current_target}")
        raw_path = self.path_planner.plan_A_Star(
            start=(actual_pos.x, actual_pos.y),
            goal=(self.current_target.x, self.current_target.y),
            map=self.map_builder
        )

        if raw_path is None:
            self.get_logger().warn("prepare_movement: No path found")
            return None

        # Optional smoothing
        self.path = self.path_planner.smooth_with_catmull_rom(raw_path, samples_per_seg=8, kappa_max=0.3)
        self.path_planner.path = self.path_planner.to_waypoints_with_theta(self.path)
        return self.path

    def target_reached(self, target: Target):
        actual_pos = self.robot_state.get_position()
        dist = math.sqrt((target.x - actual_pos.x)**2 + (target.y - actual_pos.y)**2)
        return dist <= 0.6

    def stop_robot(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)

    def follow_path_step(self, path: list[tuple[float, float]]):
        if path is None or len(path) < 2:
            return

        pos = self.robot_state.get_position()
        rx, ry, rtheta = pos.x, pos.y, pos.theta

        # 1. Find the closest point index
        dists = [math.hypot(px - rx, py - ry) for px, py in path]
        closest_idx = dists.index(min(dists))

        # 2. Find lookahead target
        target_idx = closest_idx
        for i in range(closest_idx, len(path)):
            dist_to_point = math.hypot(path[i][0] - rx, path[i][1] - ry)
            target_idx = i
            if dist_to_point >= self.min_lookahead_dist:
                break
        
        tx, ty = path[target_idx]

        # 3. Calculate global displacement and target angle
        dx_global = tx - rx
        dy_global = ty - ry
        dist_to_end = math.hypot(path[-1][0] - rx, path[-1][1] - ry)
        
        # Calculate angle to the target point
        target_angle = math.atan2(dy_global, dx_global)
        angle_to_target = self.normalize_angle(target_angle - rtheta)

        # 4. Transform to local coordinates
        lx = dx_global * math.cos(rtheta) + dy_global * math.sin(rtheta)
        ly = -dx_global * math.sin(rtheta) + dy_global * math.cos(rtheta)

        cmd = Twist()

        # 5. Logic: Turn first if the target is behind
        # Threshold: 1.57 rad is approx 90 degrees
        if abs(angle_to_target) > 1.57:
            # Target is behind us -> Only rotate, no translation
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            # Rotate towards the target angle
            cmd.angular.z = -angle_to_target * 20
            self.logger.info("Turning to face target...")
        else:
            # Target is in front hemisphere -> Normal holonomic move
            cmd.linear.x = -lx * self.k_linear 
            cmd.linear.y = -ly * self.k_linear 
            
            # Keep facing the target or use your desired_rotation
            # Here we use target_angle to stay aligned with the path
            cmd.angular.z = -angle_to_target * self.k_angular 

        # 6. Clamping
        cmd.linear.x = max(-self.max_linear_speed, min(self.max_linear_speed, cmd.linear.x))
        cmd.linear.y = max(-self.max_linear_speed, min(self.max_linear_speed, cmd.linear.y))
        cmd.angular.z = max(-self.max_angular_speed, min(self.max_angular_speed, cmd.angular.z))

        # 7. Publish
        self.cmd_vel_pub.publish(cmd)

        # 8. Arrival Logic
        if dist_to_end <= 0.04:
            self.logger.info("Path completed")
            return True 
        
        return False

    def start_control_loop(self):
        def loop():
            while rclpy.ok():   # solange ROS läuft
                self.__control_loop()
                time.sleep(self.tick_period)  # gleiche Periode wie dein Timer

        t = threading.Thread(target=loop, daemon=True)
        t.start()
        self.control_loop_thread = t

    def normalize_angle(self, a):
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
        return a

    def __control_loop(self):
        """Main robot control loop"""
        try:
            # self.strategy_engine.update_time(elapsed_time=self.tick_period)
            actual_pos = self.robot_state.get_position()
            print(f"control_loop. len(self.targets)={len(self.targets)}")
            main_strategie = self.strategy_engine.decide_next_action(robot_position=(actual_pos.x, actual_pos.y), nearby_targets=self.targets, battery_level=100)
            status_msg = String(data=f"Strategy: {main_strategie}")
            # self.status_pub.publish(status_msg)
            actual_path : list[tuple[float, float]] | None  = []

            if main_strategie.action == "return_home":
                self.logger.info(f"control_loop:: main stragie: return_home")
                actual_path = self.path_planner.plan_A_Star(start=(actual_pos.x, actual_pos.y), goal=(self.home_position.x, self.home_position.y), map=self.map_builder)
        
            else:
                action = self.action_planner.get_next_action()
                self.logger.info(f"Action{action}")
                if action is None:
                    pass
                elif action.action_type == ActionType.MOVE_TO:
                    targets: list[Target] = []
                    # pdb.set_trace()

                    if main_strategie.action == "grab_nearest":
                        self.logger.info(f"Moving to the nearest nut")
                        targets = self.targets
                    elif main_strategie.action == "explore":
                        self.logger.info(f"Exploring")
                        targets = self.aruco_map_markers
                        self.exploring = True
                        
                    actual_path = self.prepare_movement(targets=targets)
                            
                elif action.action_type == ActionType.GRAB:
                    self.action_planner.complete_current_action()
                    self.stop_robot()

                elif action.action_type == ActionType.RETURN:
                    self.stop_robot()
                    actual_path = self.prepare_movement(targets=[self.home_position])
                elif action.action_type == ActionType.RELEASE:
                    self.action_planner.complete_current_action()
                elif action.action_type == ActionType.WAIT:
                    self.action_planner.complete_current_action()
                    self.action_planner.clear_completed_actions()
                    self.action_planner.add_action(ActionType.MOVE_TO)
                    self.action_planner.add_action(ActionType.GRAB)
                    self.action_planner.add_action(ActionType.RETURN)
                    self.action_planner.add_action(ActionType.RELEASE)
                    self.action_planner.add_action(ActionType.WAIT)
                        
            if self.trajectory_follower_thread is not None and self.trajectory_follower_thread.is_alive():
                self.logger.debug("Follow path thread running")
                return
            
            if actual_path is not None and len(actual_path) >= 1: 
                self.publish_path(actual_path)
                print(f"control_loop: len(actual_path)={len(actual_path)}")             
                if self.trajectory_follower_thread is None or not self.trajectory_follower_thread.is_alive():
                    self.trajectory_follower_thread = threading.Thread(
                        target=self.follow_path_step,
                        args=(actual_path,)
                    )
                    self.trajectory_follower_thread.start()

            else:
                self.logger.info("Skipping follow path step. No enough data")

        except Exception as e:
            self.logger.error(f"Control loop error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = MAMRobotNode()
    try:
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    # finally:
    #     rclpy.shutdown()


if __name__ == '__main__':
    main()
