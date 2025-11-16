#!/usr/bin/env python3
"""
Complete ROS2 Integration Script
1. Revert broken changes
2. Fix known bugs
3. Verify tests pass
4. Add ROS2 integration
"""

import os
import subprocess
import sys
# from pathlib import Path  # Unused

class FullROS2Integration:
    def __init__(self, repo_path):
        self.repo_path = Path(repo_path)
        self.mam_python_path = self.repo_path / "mam_python"
        self.success = True
        
    def run_cmd(self, cmd, description=""):
        """Exécute une commande shell"""
        print(f"\n▶ {description}")
        print(f"  Command: {cmd}\n")
        
        result = subprocess.run(cmd, shell=True, cwd=self.repo_path)
        if result.returncode != 0:
            print(f"❌ Failed: {description}")
            self.success = False
            return False
        print(f"✓ {description}")
        return True
    
    def step_1_revert(self):
        """Step 1: Revert all broken changes"""
        print("\n" + "=" * 80)
        print("STEP 1: REVERTING BROKEN CHANGES")
        print("=" * 80)
        
        self.run_cmd("git checkout mam_python/", "Reverting mam_python/ directory")
    
    def step_2_fix_known_bugs(self):
        """Step 2: Fix known bugs"""
        print("\n" + "=" * 80)
        print("STEP 2: FIXING KNOWN BUGS")
        print("=" * 80)
        
        # Fix Tuple import in transforms.py
        transforms_file = self.mam_python_path / 'utils' / 'transforms.py'
        if transforms_file.exists():
            with open(transforms_file, 'r') as f:
                content = f.read()
            
            # Check if Tuple is used but not imported
            if 'Tuple' in content and 'from typing import' not in content:
                print("▶ Fixing missing Tuple import in transforms.py")
                # Add import at the very beginning (after docstring if exists)
                lines = content.split('\n')
                import_added = False
                
                for i, line in enumerate(lines):
                    # Skip shebang and docstrings
                    if i < 10 and (line.startswith('#') or line.startswith('"""') or line.startswith("'''")):
                        continue
                    if not import_added and not line.startswith('import ') and not line.startswith('from '):
                        if line.strip() == '':
                            continue
                        # Insert import here
                        lines.insert(i, 'from typing import Tuple')
                        import_added = True
                        break
                
                if not import_added:
                    # Add at the top
                    lines.insert(0, 'from typing import Tuple')
                
                content = '\n'.join(lines)
                with open(transforms_file, 'w') as f:
                    f.write(content)
                print("✓ Fixed Tuple import")
            elif 'from typing import' in content and 'Tuple' not in content:
                print("▶ Fixing missing Tuple in typing import")
                content = content.replace(
                    'from typing import',
                    'from typing import Tuple,'
                )
                with open(transforms_file, 'w') as f:
                    f.write(content)
                print("✓ Fixed Tuple import")
        
        return True
    
    def step_3_verify_clean_state(self):
        """Step 3: Verify clean state"""
        print("\n" + "=" * 80)
        print("STEP 3: VERIFYING CLEAN STATE")
        print("=" * 80)
        
        os.chdir(self.mam_python_path)
        
        # Check syntax
        print("\n▶ Checking Python syntax...")
        result = subprocess.run(
            "python3 -m py_compile $(find . -name '*.py' -type f | grep -v __pycache__)",
            shell=True
        )
        
        if result.returncode == 0:
            print("✓ All Python files have valid syntax")
        else:
            print("❌ Syntax errors found!")
            self.success = False
            return False
        
        return True
    
    def step_4_run_tests(self):
        """Step 4: Run tests"""
        print("\n" + "=" * 80)
        print("STEP 4: RUNNING TESTS")
        print("=" * 80)
        
        os.chdir(self.mam_python_path)
        
        print("\n▶ Running pytest...")
        result = subprocess.run("python3 -m pytest -v", shell=True)
        
        if result.returncode == 0:
            print("\n✓ All tests passed!")
            return True
        else:
            print("\n❌ Tests failed!")
            self.success = False
            return False
    
    def step_5_add_ros2_files(self):
        """Step 5: Add ROS2 integration files"""
        print("\n" + "=" * 80)
        print("STEP 5: ADDING ROS2 INTEGRATION FILES")
        print("=" * 80)
        
        # Create ros2_node.py
        ros2_wrapper = '''#!/usr/bin/env python3
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
'''
        
        ros2_path = self.mam_python_path / 'ros2_node.py'
        with open(ros2_path, 'w') as f:
            f.write(ros2_wrapper)
        print(f"✓ Created {ros2_path.name}")
        
        # Create launch directory and file
        launch_dir = self.mam_python_path / 'launch'
        os.makedirs(launch_dir, exist_ok=True)
        
        launch_file = '''#!/usr/bin/env python3
"""ROS2 Launch file for MAM Robot"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    mam_robot_node = Node(
        package='mam_python',
        executable='ros2_node.py',
        name='mam_robot_node',
        output='screen',
    )
    
    return LaunchDescription([mam_robot_node])
'''
        
        launch_path = launch_dir / 'mam_robot.launch.py'
        with open(launch_path, 'w') as f:
            f.write(launch_file)
        print(f"✓ Created {launch_path.name}")
        
        # Create package.xml
        package_xml = '''<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <n>mam_python</n>
  <version>0.1.0</version>
  <description>MAM Robot - Squirrel Cup Eurobot 2026</description>
  
  <maintainer email="team@nestmasters.dev">Nest Masters</maintainer>
  <license>MIT</license>
  
  <buildtool_depend>ament_cmake_python</buildtool_depend>
  
  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>nav_msgs</depend>
  
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
'''
        
        package_path = self.mam_python_path / 'package.xml'
        with open(package_path, 'w') as f:
            f.write(package_xml)
        print(f"✓ Created package.xml")
        
        # Create setup.py
        setup_py = '''from setuptools import setup, find_packages

setup(
    name='mam_python',
    version='0.1.0',
    packages=find_packages(),
    install_requires=['setuptools'],
    author='Nest Masters',
    description='MAM Robot - Squirrel Cup Eurobot 2026',
    license='MIT',
    entry_points={
        'console_scripts': [
            'ros2_node = mam_python.ros2_node:main',
        ],
    },
)
'''
        
        setup_path = self.mam_python_path / 'setup.py'
        with open(setup_path, 'w') as f:
            f.write(setup_py)
        print(f"✓ Created setup.py")
    
    def step_6_verify_ros2(self):
        """Step 6: Verify ROS2 files"""
        print("\n" + "=" * 80)
        print("STEP 6: VERIFYING ROS2 FILES")
        print("=" * 80)
        
        files_to_check = [
            self.mam_python_path / 'ros2_node.py',
            self.mam_python_path / 'launch' / 'mam_robot.launch.py',
            self.mam_python_path / 'package.xml',
            self.mam_python_path / 'setup.py',
        ]
        
        for file_path in files_to_check:
            if file_path.exists():
                print(f"✓ {file_path.relative_to(self.repo_path)}")
            else:
                print(f"❌ {file_path.relative_to(self.repo_path)} NOT FOUND")
                self.success = False
    
    def step_7_final_tests(self):
        """Step 7: Final test verification"""
        print("\n" + "=" * 80)
        print("STEP 7: FINAL TEST VERIFICATION")
        print("=" * 80)
        
        os.chdir(self.mam_python_path)
        
        print("\n▶ Running final pytest...")
        result = subprocess.run("python3 -m pytest -v --tb=short", shell=True)
        
        if result.returncode == 0:
            print("\n✓ All tests PASSED!")
            return True
        else:
            print("\n❌ Tests FAILED!")
            self.success = False
            return False
    
    def run(self):
        """Execute all steps"""
        print("\n" + "=" * 80)
        print("COMPLETE ROS2 INTEGRATION PIPELINE")
        print("=" * 80)
        
        self.step_1_revert()
        self.step_2_fix_known_bugs()
        self.step_3_verify_clean_state()
        self.step_4_run_tests()
        
        if not self.success:
            print("\n❌ Integration failed at verification step!")
            return False
        
        self.step_5_add_ros2_files()
        self.step_6_verify_ros2()
        self.step_7_final_tests()
        
        if self.success:
            print("\n" + "=" * 80)
            print("✅ INTEGRATION SUCCESSFUL!")
            print("=" * 80)
            print("""
Your robot is now ROS2 ready! Next steps:

1. Commit to GitHub:
   cd ~/squirrelcup
   git add -A
   git commit -m "Add ROS2 Humble integration"
   git push origin nicolas_python_core

2. On your PC with GPU:
   cd ~/colcon_ws/src
   git clone https://github.com/NicolasPh7/squirrelcup.git
   cd ~/colcon_ws
   colcon build
   source install/setup.bash
   ros2 launch mam_python mam_robot.launch.py

Ready to dominate the competition! 🚀
""")
            return True
        else:
            print("\n❌ Integration completed with errors!")
            return False


if __name__ == "__main__":
    if len(sys.argv) > 1:
        repo_path = sys.argv[1]
    else:
        repo_path = os.path.expanduser('~/squirrelcup')
    
    integrator = FullROS2Integration(repo_path)
    success = integrator.run()
    sys.exit(0 if success else 1)