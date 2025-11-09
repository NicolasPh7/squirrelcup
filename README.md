# MAM Python - Eurobot 2026 Robot Control

This is the main control architecture for the Squirrel Cup robot project. The goal is to build a robot that collects hazelnut crates in 90 seconds and stores them in its nest.

## Project Structure

```
mam_python/
├── control/              # Motor and actuator control
│   ├── motion_controller.py      # Movement commands
│   └── arm_controller.py         # Robotic arm and gripper
├── core/                 # Core system state
│   ├── game_state.py             # Game state tracking
│   └── robot_state.py            # Robot telemetry and config
├── perception/           # Sensor data processing
│   ├── map_builder.py            # Environment mapping
│   ├── object_manager.py         # Object detection tracking
│   └── target_selector.py        # Target prioritization
├── planning/             # Path and trajectory planning
│   ├── path_planner.py           # Path generation
│   ├── collision_checker.py      # Obstacle avoidance
│   └── trajectory_generator.py   # Smooth trajectory curves
├── decision/             # Decision making system
│   ├── action_planner.py         # Action queue management
│   ├── mission_manager.py        # Mission tracking
│   └── strategy_engine.py        # Strategy selection
├── scoring/              # Points and scoring
│   ├── score_tracker.py          # Real-time score tracking
│   ├── score_predictor.py        # Score prediction
│   └── score_optimizer.py        # Point optimization
├── utils/                # Utility functions
│   ├── logger.py                 # Logging system
│   ├── geometry.py               # Geometry math
│   └── transforms.py             # 3D transformations
├── tests/                # Unit tests
│   ├── test_planning.py
│   ├── test_scoring.py
│   └── test_strategy.py
└── __init__.py           # Module exports
```

## Quick Start

```bash
git clone https://github.com/NicolasPh7/squirrelcup.git
cd squirrelcup
```

## Basic Usage

```python
from mam_python import (
    GameState, RobotState,
    MotionController, ArmController,
    PathPlanner, CollisionChecker,
    StrategyEngine, ScoreTracker
)

# Initialize game
game = GameState(match_duration=90)
game.start_match()

# Setup robot
robot = RobotState()
motion = MotionController(max_speed=1.0)
arm = ArmController()

# Plan a path
planner = PathPlanner()
waypoints = planner.plan_line(0, 0, 1, 1, num_points=10)

# Check collisions
collision = CollisionChecker(robot_radius=0.2)
collision.add_obstacle(0.5, 0.5, radius=0.1)

# Make decisions
strategy = StrategyEngine()
decision = strategy.decide_next_action(
    robot_position=(0, 0),
    nearby_targets=[],
    battery_level=100
)

# Track score
scorer = ScoreTracker()
scorer.add_points(100, "crate_collected")
print(f"Score: {scorer.get_total_score()}")
```

## Module Details

### Control

**MotionController** - Drive commands

```python
motion = MotionController(max_speed=1.0)
motion.forward(0.8)
motion.turn_left(0.5)
motion.arc(angle=45, radius=0.5)
motion.stop()
```

**ArmController** - Arm and gripper

```python
arm = ArmController()
arm.open_gripper()
arm.close_gripper(force=100)
arm.set_arm_angle(90)
arm.grab_crate()
arm.release_crate()
```

### Core

**GameState** - Game tracking

```python
game = GameState(match_duration=90)
game.start_match()
game.add_crate(crate_id=1, x=0.5, y=0.5)
game.collect_crate(1)
game.update_score(100)
elapsed = game.get_elapsed_time()
```

**RobotState** - Robot status

```python
robot = RobotState()
robot.update_position(x=1.0, y=2.0, theta=0.785)
robot.set_mode(RobotMode.MOVING)
robot.update_battery(delta_time=1.0)
battery = robot.battery_level
```

### Perception

**MapBuilder** - Environment grid

```python
map_builder = MapBuilder(width=3.0, height=2.0, resolution=0.05)
map_builder.update_cell(1.0, 1.0, occupied=True)
is_occupied = map_builder.is_occupied(1.0, 1.0)
free = map_builder.get_free_neighbors(0.5, 0.5)
```

**ObjectManager** - Object tracking

```python
objects = ObjectManager()
obj_id = objects.add_object(ObjectType.CRATE, x=1.0, y=1.5, confidence=0.9)
objects.update_object(obj_id, x=1.1, y=1.5)
nearest = objects.get_nearest_object(0, 0)
```

**TargetSelector** - Choose targets

```python
selector = TargetSelector()
best = selector.distance_priority(targets, robot_x=0, robot_y=0)
best = selector.score_priority(targets)
```

### Planning

**PathPlanner** - Generate paths

```python
planner = PathPlanner()
waypoints = planner.plan_line(0, 0, 3, 2, num_points=15)
arc = planner.plan_arc(0, 0, 0.5, 0, 3.14, num_points=20)
```

**CollisionChecker** - Avoid obstacles

```python
checker = CollisionChecker(robot_radius=0.2)
checker.add_obstacle(x=1.0, y=1.0, radius=0.3)
safe = not checker.check_collision(0.5, 0.5)
```

**TrajectoryGenerator** - Smooth curves

```python
gen = TrajectoryGenerator(dt=0.1)
traj = gen.generate_trajectory(waypoints, max_speed=1.0)
for pt in traj:
    print(f"x={pt.x}, y={pt.y}, v={pt.v}")
```

### Decision

**ActionPlanner** - Action queue

```python
planner = ActionPlanner()
planner.add_action(ActionType.MOVE_TO, {"x": 1.0, "y": 2.0}, priority=10)
next_action = planner.get_next_action()
planner.complete_current_action()
```

**MissionManager** - Mission tracking

```python
manager = MissionManager()
mid = manager.create_mission("Collect crates", priority=10)
manager.start_mission(mid)
manager.complete_mission(mid)
```

**StrategyEngine** - Decision engine

```python
engine = StrategyEngine()
engine.update_time(elapsed_time=30, total_time=90)
decision = engine.decide_next_action((0, 0), [], 75)
print(f"Action: {decision.action}")
```

### Scoring

**ScoreTracker** - Points tracking

```python
tracker = ScoreTracker()
tracker.add_points(100, "crate_collected")
tracker.set_multiplier(1.5)
total = tracker.get_total_score()
```

**ScorePredictor** - Predict final score

```python
predictor = ScorePredictor()
cons, opt = predictor.predict_final_score(5, 30, 90)
```

**ScoreOptimizer** - Maximize points

```python
optimizer = ScoreOptimizer()
optimizer.add_crate_value(1, base_points=100)
optimizer.update_bonuses(time_remaining=20, total_time=90)
order = optimizer.get_optimal_collection_order()
```

### Utils

**Logger** - Event logging

```python
from mam_python.utils import Logger

Logger.info("Robot started")
Logger.debug("Position: 1.0, 2.0")
Logger.warning("Low battery!")
```

**GeometryUtils** - Math helpers

```python
from mam_python.utils import Point

p1 = Point(0, 0)
p2 = Point(1, 1)
dist = p1.distance_to(p2)
angle = p1.angle_to(p2)
```

**TransformUtils** - Rotation conversions

```python
from mam_python.utils import TransformUtils

roll, pitch, yaw = TransformUtils.quaternion_to_euler(0, 0, 0.707, 0.707)
qx, qy, qz, qw = TransformUtils.euler_to_quaternion(0, 0, 1.57)
```

## Running Tests

```bash
cd mam_python/tests
python -m unittest discover
```

## Branch Info

This code is on branch `nicolas_python_core`
