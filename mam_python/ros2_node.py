#!/usr/bin/env python3
"""
ROS2 Node Wrapper for MAM Python Robot
Integrates all robot modules with ROS2 Humble and Gazebo
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String, Int32
from sensor_msgs.msg import LaserScan, JointState, Image
from nav_msgs.msg import Path

# Import local modules
from mam_python.decision import StrategyEngine, MissionManager, ActionPlanner
from mam_python.planning import PathPlanner, CollisionChecker, TrajectoryGenerator
from mam_python.control import MotionController, ArmController
from mam_python.perception import MapBuilder, ObjectManager, TargetSelector
from mam_python.scoring import ScoreTracker, ScoreOptimizer, ScorePredictor
from mam_python.core import RobotState, GameState
from mam_python.utils import Logger


class MAMRobotNode(Node):
    """Main ROS2 Node for MAM Robot"""
    
    def __init__(self):
        super().__init__('mam_robot_node')
        self.logger = Logger.get_logger('MAMRobotNode')
        
        # Initialize all robot modules
        self.robot_state = RobotState()
        self.game_state = GameState()
        
        self.strategy_engine = StrategyEngine()
        self.mission_manager = MissionManager()
        self.action_planner = ActionPlanner()
        
        self.path_planner = PathPlanner()
        self.collision_checker = CollisionChecker()
        self.trajectory_generator = TrajectoryGenerator()
        
        self.motion_controller = MotionController()
        self.arm_controller = ArmController()
        
        self.map_builder = MapBuilder()
        self.object_manager = ObjectManager()
        self.target_selector = TargetSelector()
        
        self.score_tracker = ScoreTracker()
        self.score_optimizer = ScoreOptimizer()
        self.score_predictor = ScorePredictor()
        
        # Create publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/robot/cmd_vel', 10)
        self.arm_cmd_pub = self.create_publisher(JointState, '/robot/arm_commands', 10)
        self.path_pub = self.create_publisher(Path, '/robot/planned_path', 10)
        self.score_pub = self.create_publisher(Int32, '/robot/score', 10)
        self.status_pub = self.create_publisher(String, '/robot/status', 10)
        
        # Create subscribers
        self.lidar_sub = self.create_subscription(LaserScan, '/lidar/scan', self.lidar_callback, 10)
        self.camera_sub = self.create_subscription(Image, '/camera/image', self.camera_callback, 10)
        self.odom_sub = self.create_subscription(PoseStamped, '/odometry/pose', self.odom_callback, 10)
        
        # Control loop timer (10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.logger.info("MAM Robot Node initialized!")
    
    def lidar_callback(self, msg: LaserScan):
        """Process LIDAR data"""
        try:
            self.map_builder.update_from_lidar(msg)
        except Exception as e:
            self.logger.error(f"LIDAR error: {e}")
    
    def camera_callback(self, msg: Image):
        """Process camera data"""
        try:
            objects = self.object_manager.detect_objects(msg)
            self.target_selector.update_targets(objects, self.robot_state.position)
        except Exception as e:
            self.logger.error(f"Camera error: {e}")
    
    def odom_callback(self, msg: PoseStamped):
        """Update odometry"""
        try:
            self.robot_state.update_position(msg.pose.position)
        except Exception as e:
            self.logger.error(f"Odom error: {e}")
    
    def control_loop(self):
        """Main robot control loop"""
        try:
            strategy = self.strategy_engine.get_current_strategy()
            status_msg = String(data=f"Strategy: {strategy}")
            self.status_pub.publish(status_msg)
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
