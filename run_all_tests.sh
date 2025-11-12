#!/bin/bash

echo "================================================================================"
echo "COMPLETE TEST SUITE FOR ROS2 INTEGRATION"
echo "================================================================================"

# OPTION 1: Unit tests
echo -e "\n=== OPTION 1: Unit Tests ==="
cd ~/squirrelcup/mam_python
python3 -m pytest -v
if [ $? -ne 0 ]; then
    echo "❌ Unit tests FAILED"
    exit 1
fi
echo "✅ Unit tests PASSED"

# OPTION 2: ROS2 node syntax
echo -e "\n=== OPTION 2: ROS2 Node Syntax ==="
python3 -m py_compile ~/squirrelcup/mam_python/ros2_node.py
if [ $? -ne 0 ]; then
    echo "❌ ROS2 node syntax FAILED"
    exit 1
fi
echo "✅ ROS2 node syntax OK"

# OPTION 3: Launch file syntax
echo -e "\n=== OPTION 3: Launch File Syntax ==="
python3 -m py_compile ~/squirrelcup/mam_python/launch/mam_robot.launch.py
if [ $? -ne 0 ]; then
    echo "❌ Launch file syntax FAILED"
    exit 1
fi
echo "✅ Launch file syntax OK"

# OPTION 4: Mock test
echo -e "\n=== OPTION 4: Mock Test (ROS2 Integration) ==="

# Create temp mock test file
cat > /tmp/test_ros2_mock.py << 'MOCKTEST'
#!/usr/bin/env python3
import sys
sys.path.insert(0, '/home/nico/squirrelcup')

def test_imports():
    print("▶ Testing imports...")
    try:
        from mam_python.decision import StrategyEngine, MissionManager, ActionPlanner
        from mam_python.planning import PathPlanner, CollisionChecker, TrajectoryGenerator
        from mam_python.control import MotionController, ArmController
        from mam_python.perception import MapBuilder, ObjectManager, TargetSelector
        from mam_python.scoring import ScoreTracker, ScoreOptimizer, ScorePredictor
        from mam_python.core import RobotState, GameState
        from mam_python.utils import Logger, GeometryUtils, TransformUtils
        print("✅ All imports successful")
        return True
    except Exception as e:
        print(f"❌ Import failed: {e}")
        return False

def test_module_instantiation():
    print("\n▶ Testing module instantiation...")
    try:
        from mam_python.decision import StrategyEngine, MissionManager, ActionPlanner
        from mam_python.planning import PathPlanner, CollisionChecker, TrajectoryGenerator
        from mam_python.control import MotionController, ArmController
        from mam_python.perception import MapBuilder, ObjectManager, TargetSelector
        from mam_python.scoring import ScoreTracker, ScoreOptimizer, ScorePredictor
        from mam_python.core import RobotState, GameState
        
        robot_state = RobotState()
        game_state = GameState()
        strategy_engine = StrategyEngine()
        mission_manager = MissionManager()
        action_planner = ActionPlanner()
        path_planner = PathPlanner()
        collision_checker = CollisionChecker()
        trajectory_generator = TrajectoryGenerator()
        motion_controller = MotionController()
        arm_controller = ArmController()
        map_builder = MapBuilder()
        object_manager = ObjectManager()
        target_selector = TargetSelector()
        score_tracker = ScoreTracker()
        score_optimizer = ScoreOptimizer()
        score_predictor = ScorePredictor()
        
        print("✅ All modules instantiated successfully")
        return True
    except Exception as e:
        print(f"❌ Instantiation failed: {e}")
        import traceback
        traceback.print_exc()
        return False

def test_ros2_files():
    print("\n▶ Testing ROS2 file structure...")
    try:
        from pathlib import Path
        mam_python_path = Path('/home/nico/squirrelcup/mam_python')
        
        files = [
            mam_python_path / 'ros2_node.py',
            mam_python_path / 'launch' / 'mam_robot.launch.py',
            mam_python_path / 'package.xml',
            mam_python_path / 'setup.py',
        ]
        
        for file_path in files:
            if not file_path.exists():
                print(f"❌ Missing {file_path.name}")
                return False
            print(f"  ✓ {file_path.name}")
        
        print("✅ All ROS2 files present")
        return True
    except Exception as e:
        print(f"❌ ROS2 structure check failed: {e}")
        return False

if __name__ == "__main__":
    results = []
    results.append(test_imports())
    results.append(test_module_instantiation())
    results.append(test_ros2_files())
    
    if all(results):
        print("\n" + "=" * 80)
        print("✅ ALL MOCK TESTS PASSED!")
        print("=" * 80)
        sys.exit(0)
    else:
        print("\n❌ Some mock tests failed!")
        sys.exit(1)
MOCKTEST

python3 /tmp/test_ros2_mock.py
if [ $? -ne 0 ]; then
    echo "❌ Mock test FAILED"
    exit 1
fi

echo -e "\n================================================================================"
echo "✅ ALL TESTS PASSED!"
echo "================================================================================"
echo ""
echo "Your robot is ready! Next steps:"
echo "1. Commit changes:"
echo "   git add -A"
echo "   git commit -m 'Add ROS2 integration with tests'"
echo "   git push origin nicolas_python_core"
echo ""
echo "2. On your PC with GPU:"
echo "   colcon build"
echo "   source install/setup.bash"
echo "   ros2 launch mam_python mam_robot.launch.py"
echo ""