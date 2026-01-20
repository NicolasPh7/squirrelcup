#!/usr/bin/env python3

"""
ROS2 Node Wrapper for MAM Python Robot
Integrates all robot modules with ROS2 Humble and Gazebo
"""
import pdb

import math
import threading

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
        
        self.motion_controller = MotionController()
        self.arm_controller = ArmController()
        
        self.map_builder = MapBuilder(resolution=0.1)
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

        self.tick_period = 2
        
        # Create publishers
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
        
        self.timer = self.create_timer(self.tick_period, self.control_loop)
        self.path_thread: threading.Thread | None = None
        
        self.logger.info("MAM Robot Node initialized!")
    
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
            if self.map_builder.update_from_marker_array(msg, self.robot_state.get_position()):
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
            self.get_logger().info(f"Updated robot position {self.robot_state.get_position()}")

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

        # Ersetze die alte Liste durch die neue
        self.targets = updated_targets
        self.get_logger().info(f"Targets updated: {len(self.targets)} items")
    
    def prepare_movement(self,  targets: list[Target]) -> list[tuple[float, float]] | None:
        # pdb.set_trace()
        actual_pos = self.robot_state.get_position()
        target = self.target_selector.distance_priority(targets, actual_pos.x, actual_pos.y)

        if self.current_target is None: 
            if target is None: 
                return
            self.current_target = target # If no current target, take the given target
        else:
            if target.id == self.current_target.id: #Update current target with updated position
                self.current_target = target

        if self.target_reached(self.current_target):
            self.stop_robot()
            self.action_planner.complete_current_action()
            self.current_target = None
            self.logger.info(f"Target reached")
            return None
        else:
            self.logger.info(f"Planned path to nearest target:{self.current_target}")
            return self.path_planner.plan_A_Star(start=(actual_pos.x, actual_pos.y), goal=(self.current_target.x, self.current_target.y), map=self.map_builder)

    def target_reached(self, target: Target):
        actual_pos = self.robot_state.get_position()
        dist = math.sqrt((target.x - actual_pos.x)**2 + (target.y - actual_pos.y)**2)
        return dist < 0.2

    def stop_robot(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)


    def follow_pure_pursuit_step(self, path, look_ahead_dist=0.3, max_vel=0.5):
        """
        Pure Pursuit controller for a holonomic robot.
        
        Args:
            path: List of (x, y) tuples representing the global path.
            look_ahead_dist: How far ahead the robot looks (tuning parameter).
            max_vel: Maximum linear velocity.
        """
        actual_pos = self.robot_state.get_position()
        robot_x, robot_y = actual_pos.x, actual_pos.y
        theta = actual_pos.theta

        # 1. Find the look-ahead point
        # We look for the first point in the path that is further than look_ahead_dist
        target_point = None
        for i in range(len(path)):
            dist = math.hypot(path[i][0] - robot_x, path[i][1] - robot_y)
            if dist > look_ahead_dist:
                target_point = path[i]
                break
        
        # If no point is far enough, aim for the final destination
        if target_point is None:
            target_point = path[-1]

        # 2. Calculate Global Error Vector
        dx_global = target_point[0] - robot_x
        dy_global = target_point[1] - robot_y
        distance_to_target = math.hypot(dx_global, dy_global)

        # 3. Transform to Local Robot Frame
        # (So the robot knows how much to move forward vs sideways)
        local_x = dx_global * math.cos(theta) + dy_global * math.sin(theta)
        local_y = -dx_global * math.sin(theta) + dy_global * math.cos(theta)

        # 4. Generate Twist Command
        cmd = Twist()
        
        # Stop condition: if we are close to the final point
        final_dist = math.hypot(path[-1][0] - robot_x, path[-1][1] - robot_y)
        if final_dist < 0.05:
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            self.cmd_vel_pub.publish(cmd)
            return True # Path finished

        # Normalize the local vector and scale by max velocity
        # This keeps the robot moving at a constant speed along the path
        look_ahead_norm = math.hypot(local_x, local_y)
        cmd.linear.x = (local_x / look_ahead_norm) * max_vel
        cmd.linear.y = (local_y / look_ahead_norm) * max_vel

        # 5. Optional: Keep robot facing the direction of travel
        target_angle = math.atan2(dy_global, dx_global)
        angle_error = target_angle - theta
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))
        cmd.angular.z = 1.5 * angle_error # P-gain for rotation

        self.cmd_vel_pub.publish(cmd)
        return False

    def control_loop(self):
        """Main robot control loop"""
        try:
            self.strategy_engine.update_time(elapsed_time=self.tick_period)
            actual_pos = self.robot_state.get_position()
            main_strategie = self.strategy_engine.decide_next_action(robot_position=(actual_pos.x, actual_pos.y), nearby_targets=self.targets, battery_level=100)
            status_msg = String(data=f"Strategy: {main_strategie}")
            # self.status_pub.publish(status_msg)
            actual_path : list[tuple[float, float]] | None  = []

            if main_strategie.action == "return_home":
                actual_path = self.path_planner.plan_A_Star(start=(actual_pos.x, actual_pos.y), goal=(self.home_position.x, self.home_position.y), map=self.map_builder)
        
            else:
                action = self.action_planner.get_next_action()
                self.logger.info(f"Action{action}")
                if action is None:
                    pass
                elif action.action_type == ActionType.MOVE_TO:
                    targets: list[Target] = []

                    if main_strategie.action == "grab_nearest":
                        self.logger.info(f"Moving to the nearest nut")
                        targets = self.targets
                    elif main_strategie.action == "explore":
                        self.logger.info(f"Exploring")
                        targets = self.aruco_map_markers
                        
                    actual_path = self.prepare_movement(targets=targets)
                            
                elif action.action_type == ActionType.GRAB:
                    self.action_planner.complete_current_action()
                elif action.action_type == ActionType.RETURN:
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
                        
            if self.path_thread is not None and self.path_thread.is_alive():
                self.logger.debug("Follow path thread running")
                return
            
            if actual_path is not None and len(actual_path) >= 1: 
                self.publish_path(actual_path)
                step: tuple[float, float] = actual_path[0]
                # self.follow_path_step(step[0], step[1])
                # t = threading.Thread(
                #     target=self.follow_pure_pursuit_step,
                #     args=(actual_path)
                # )
                # t.start()
            else:
                self.logger.info("Skipping follow path step. No enough data")

        except Exception as e:
            self.logger.error(f"Control loop error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = MAMRobotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    # finally:
    #     rclpy.shutdown()


if __name__ == '__main__':
    main()
