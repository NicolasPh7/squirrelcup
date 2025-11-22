#!/usr/bin/env python3

"""
ROS2 Node Wrapper for MAM Python Robot
Integrates all robot modules with ROS2 Humble and Gazebo
"""
import pdb

import math
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
        self.aruco_map_markers = [
            Target(id=20, x=0.6, y=1.4),
            Target(id=21, x=2.4, y=1.4),
            Target(id=22, x=0.6, y=0.6),
            Target(id=23, x=2.4, y=0.6),
        ]

        self.score_tracker = ScoreTracker()
        self.score_optimizer = ScoreOptimizer()
        self.score_predictor = ScorePredictor()

        self.tick_period = 0.1 
        
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
            self.map_builder.update_from_marker_array(msg)
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

        except Exception as e:
            self.logger.error(f"Odom error: {e}")

    def nuts_callback(self, msg: MarkerArray):
        """Update targets list from incoming MarkerArray."""
        updated_targets: List[Target] = []

        for marker in msg.markers:
            if marker.action == marker.DELETEALL: 
                return
            
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

        if target is not None:
            if self.target_reached(target):
                self.stop_robot()
                self.action_planner.complete_current_action()
                self.logger.info(f"Target reached")
                return None
            else:
                self.logger.info(f"Planned path to nearest target:{target}")
                return self.path_planner.plan_A_Star(start=(actual_pos.x, actual_pos.y), goal=(target.x, target.y), map=self.map_builder)

    def target_reached(self, target: Target):
        actual_pos = self.robot_state.get_position()
        dist = math.sqrt((target.x - actual_pos.x)**2 + (target.y - actual_pos.y)**2)
        return dist < 0.1

    def stop_robot(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)


    def follow_path_step(self, goal_x, goal_y, tolerance: float = 0.001):
        """
        Folgt einem geplanten Pfad, indem nacheinander die Zielpunkte angefahren werden.
        
        Args:
            path_points: Liste von (x, y) Koordinaten in Weltkoordinaten
            tolerance: Abstand, ab dem ein Punkt als erreicht gilt
        """
        cmd = Twist()

        # Aktuelle Pose
        actual_pos = self.robot_state.get_position()
        x = actual_pos.x
        y = actual_pos.y
        theta = actual_pos.theta

        # Fehler
        dx = goal_x - x
        dy = goal_y - y
        distance = math.hypot(dx, dy)
        target_angle = math.atan2(dy, dx)
        angle_error = target_angle - theta

        # Normalisiere Winkel
        while angle_error > math.pi:
            angle_error -= 2*math.pi
        while angle_error < -math.pi:
            angle_error += 2*math.pi

        self.get_logger().info(f"Path Step: distance: {distance}, angle_error {angle_error}")

        # Steuerung (einfacher P-Regler)
        if distance > tolerance:
            cmd.linear.x = 10 * min(0.05, distance)     # Geschwindigkeit begrenzen, factor depends on map resolution!!!
            # cmd.linear.x = 0.5
            # cmd.angular.z = 0.5 * angle_error       # Drehen Richtung Ziel
            cmd.angular.z = angle_error       # Drehen Richtung Ziel
        # else:
        #     cmd.linear.x = 0.0
        #     cmd.angular.z = 0.0

        # Publiziere Steuerkommando
        self.cmd_vel_pub.publish(cmd)
            

        # # Stop am Ende
        # cmd.linear.x = 0.0
        # cmd.angular.z = 0.0
        # self.cmd_vel_pub.publish(cmd)
        # self.get_logger().info("Path Step completed")

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
                        
            if actual_path is not None and len(actual_path) >= 1: 
                self.publish_path(actual_path)
                step: tuple[float, float] = actual_path[0]
                self.follow_path_step(step[0], step[1])

        except Exception as e:
            self.logger.error(f"Control loop error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = MAMRobotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
