# Eurobot 2026 – SquirrelCup Robot

**Repository**: `squirrelcup` | **Team**: MAM (Mechanical & Autonomous Mobile)  
**Competition**: Eurobot 2026 – Autonomous robot collecting & delivering objects on 8×6 chessboard arena  
**Last Updated**: January 2026

---

## Quick Overview

This project is a **competitive robotics platform for Eurobot 2026**. The robot must autonomously collect objects (nuts) and deliver them to designated zones to score points.

We use a **dual-stream development approach**:
- **`main`**: Pure simulation (no ROS2) for rapid algorithm prototyping
- **`dev`**: Production ROS2 stack for realistic testing with Gazebo
- **`feat_*`**: Experimental feature branches (new capabilities, optimizations)

---

## Repository Structure

```
squirrelcup/
├── eurobot2026-strategy-sim/          [main, dev] Standalone simulator
│   ├── src/
│   │   ├── ai_decision_demo.cpp       C++/SFML interactive grid visualizer
│   │   ├── ai_decision_demo.py        Python Monte Carlo planner + Matplotlib
│   │   └── chessboard_map.py          Static arena map reference
│   └── assets/
│       ├── table_bis.png              Arena background
│       └── DejaVuSans.ttf             Font for rendering
│
├── mam_eurobot_2026/                  [dev, feat_*] Core ROS2 package
│   ├── src/
│   │   ├── inertial_odometry.cpp      Velocity integration → pose (RK2 method)
│   │   ├── nut_identifier2.cpp        Vision: nut detection (OpenCV HSV)
│   │   ├── cpp_test.cpp               Test node: circular motion
│   │   └── teleop.cpp                 Manual keyboard control
│   ├── launch/
│   │   └── arena.launch.py            Gazebo + ROS2 nodes startup
│   ├── test/
│   │   ├── test_odometry.py           Unit tests: odometry math
│   │   └── test_integration_odometry.py E2E test: robot reaches target
│   ├── worlds/
│   │   └── arena_world.sdf            Gazebo world (8×6 arena, physics)
│   └── models/
│       ├── simple_robot/              Robot URDF/SDF
│       ├── crate/                     Collectible object model
│       └── gripper/                   End-effector gripper
│
├── mam_python/                        [feat_ros_python_core_integration] Decision core
│   ├── decision/                      Strategy logic (ported from main)
│   ├── planning/                      Pathfinding (A*, RRT)
│   ├── control/                       Velocity command generation
│   ├── perception/                    Object detection & tracking
│   └── scoring/                       Points optimization
│
├── robot_v3/                          [feat_ros_python_core_integration] Mechanical v3
├── mechanum_wheels/                   [feat_ros_python_core_integration] Mecanum drive variant
│
├── .devcontainer/                     Docker development container
├── .github/workflows/
│   └── ci.yml                         GitHub Actions CI/CD
└── tools/
    └── run_tests_in_container.sh      Test runner for CI
```

---

## All Branches Explained

### 🟢 `main` – Strategy & Simulation (STABLE)

**Purpose**: Pure algorithm testing on 8×6 grid, no ROS2 overhead  
**Use case**: Prototype decision logic (A* pathfinding, Monte Carlo planning)  

**How to use**:
```bash
cd eurobot2026-strategy-sim
cmake -B build && cmake --build build
./build/ai_decision_demo           # Interactive SFML visualizer
# OR
python src/ai_decision_demo.py     # Monte Carlo planner with live graphs
```

**Key files**:
- `ai_decision_demo.cpp`: Interactive grid with A* solver, colored decision branches
- `ai_decision_demo.py`: Monte Carlo Tree Search evaluation of actions, live bar chart
- `chessboard_map.py`: Static map display for reference

**Dependencies**: SFML 2.5, matplotlib, numpy, OpenCV (minimal)

**Development pattern**:
1. Modify strategy in `ai_decision_demo.py` (edit `random_policy()`, `best_action_search()`)
2. Re-run Python version immediately (no compilation)
3. Validate behavior visually
4. Commit with clear strategy description

---

### 🟢 `dev` – ROS2 Production Stack (STABLE)

**Purpose**: Full ROS2 Humble integration with realistic Gazebo simulation  
**Use case**: Test complete robot stack with sensors, odometry, vision  

**How to use**:
```bash
source /opt/ros/humble/setup.bash
cd ~/eurobot_2026_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch mam_eurobot_2026 arena.launch.py
```

**Architecture**:
```
Gazebo Physics Engine
    ↓ [ros_gz_bridge]
/cmd_vel ──→ inertial_odometry ──→ /odom (200 Hz)
             ↓ [TF broadcaster]
/camera ──→ nut_identifier2 ──→ /nut_detections
             ↓ [future: mam_python wrapper]
```

**Key nodes**:
- `inertial_odometry`: Integrates velocity commands → position using RK2 (Runge-Kutta 2)
- `nut_identifier2`: Processes camera feed, detects nuts via OpenCV (HSV color space)
- `cpp_test`, `py_test`: Test nodes that publish `/cmd_vel` commands

**Run tests**:
```bash
colcon test --packages-select mam_eurobot_2026
```

**Dependencies**: ROS2 Humble, Gazebo Ignition, OpenCV, tf2, colcon

---

### 🟡 `feat_ros_python_core_integration` – Python Decision Core (BETA)

**Purpose**: Integrate strategy logic from `main` into ROS2 as reusable Python modules  
**Use case**: Rapid iteration on decision logic without C++ recompilation  

**Key additions**:

**New package `mam_python/`**:
- `decision/strategy_engine.py`: Ported strategy from main (where to go, what to do)
- `planning/path_planner.py`: A* and other pathfinding algorithms
- `control/motion_controller.py`: Generates `/cmd_vel` Twist messages
- `perception/object_manager.py`: Tracks detected nuts, manages world state
- `scoring/`: Calculates expected points, ranks actions

**Mechanical variants**:
- `robot_v3/`: Updated URDF (v3 geometry, joint mapping)
- `mechanum_wheels/`: Mecanum wheel kinematics and control

**New C++ nodes**:
- `aruco_localization.cpp`: Visual localization using ArUco fiducial markers
- `object_detector.cpp`: Unified detector for nuts + obstacles
- `robot_controller.cpp`: Main coordinator bridging strategy → hardware commands

**New config files**:
- `config/aruco_tags.yaml`: Fiducial marker definitions & world poses
- `config/crates.yaml`: Nut detection parameters (HSV ranges, morphology)
- `config/fix_balises.yaml`: Fixed navigation beacon locations
- `config/ros_gz_bridge.yaml`: ROS2 ↔ Gazebo topic mapping

**Development workflow**:
1. Develop strategy logic in `mam_python/decision/` (pure Python, no ROS2 required)
2. Wrap with `ros2_node.py` to expose as ROS2 node
3. Launch with `mam_python_core.launch.py`
4. Strategy reads `/odom` + `/nut_detections`, publishes `/cmd_vel`
5. Test in closed loop with Gazebo

**Benefits**:
- No C++ recompilation (fast iteration)
- Modular design (test each component separately)
- Pure Python testing (unit tests via pytest)

---

### 🟡 `feat_trajectory_planner` – Advanced Trajectory Planning (WIP)

**Purpose**: Dedicated trajectory planning with smooth paths and obstacle avoidance  
**Use case**: Optimal motion planning (RRT*, Theta*, velocity profiling)  

**Expected components**:
- `src/trajectory_planner.hpp`: Reusable planning header
- `src/trajectory_planner_node.cpp`: ROS2 service `/plan_trajectory`
- Algorithms:
  - **RRT\***: Asymptotically optimal randomized planning
  - **Theta\***: Any-angle pathfinding (faster than RRT)
  - **Velocity profiling**: Trapezoid acceleration/deceleration (prevent wheel slip)
  - **Spline smoothing**: Cubic Hermite or Bezier for smooth curves

**Integration**: Service-based architecture
```bash
ros2 service call /plan_trajectory nav_msgs/srv/GetPlan \
  "start: {pose: {position: {x: 0, y: 0}}}" \
  "goal: {pose: {position: {x: 0.8, y: 0.8}}}"
```

**Status**: WIP – algorithm design phase

---

### 🟡 `feat_model_3` – Mechanical Refinement (WIP)

**Purpose**: Update robot mechanics, physics tuning, CAD improvements  
**Use case**: Improve simulation realism, fine-tune friction/inertia  

**Expected updates**:
- `worlds/arena_world.sdf`: Physics parameters (timestep, gravity, contact stiffness)
- `models/simple_robot/`: Updated URDF with refined mass distribution
- `models/gripper/`: CAD mesh improvements, joint limit adjustments
- Robot dynamics: Inertia tensors, friction coefficients

**Typical tweaks**:
- ODE solver parameters (for stability)
- Wheel friction coefficients (affects traction)
- Mass distribution (affects dynamic behavior)
- Contact material properties (Gazebo physics)

**Status**: WIP – mechanical design iteration

---

### 🔴 `nicolas_python_core` – Personal Sandbox (EXPERIMENTAL)

**Purpose**: Personal experimentation branch for strategy variants  
**Use case**: Develop alternative approaches without affecting main teams

**Typical workflow**:
1. Fork from `feat_ros_python_core_integration` or `dev`
2. Experiment with custom strategy logic
3. If successful, propose PR back to feature branch
4. If unsuccessful, keep as personal reference

**Note**: Not merged into main branches unless validated

---

## Arena & Algorithm Details

### Arena Model
- **Grid**: 8 columns × 6 rows (100mm × 100mm cells)
- **Key zones**:
  - **Yellow Nest** (0,0): Drop zone for yellow team
  - **Blue Nest** (7,0): Drop zone for blue team
  - **Collection** (0,3): Source of nuts to collect
  - **Pantry** (7,3): Secondary resource zone
  - **Thermo** (4,5): Bonus objective (temperature sensor)

### Core Algorithms

**A* Pathfinding** (used in `ai_decision_demo.cpp`):
- Finds shortest path on 8×6 grid
- 4-connectivity (up/down/left/right, no diagonals)
- Manhattan distance heuristic
- O(N log N) complexity

**Monte Carlo Planner** (used in `ai_decision_demo.py`):
- Stochastic action evaluation
- Biased random policy (75% toward objective)
- UCB-like exploration (balance exploitation vs. exploration)
- Incremental aggregation (scores accumulate across frames)

**RK2 Odometry** (used in `inertial_odometry.cpp`):
- Runge-Kutta 2nd order integration
- Evaluates velocity at midpoint for better accuracy
- 200 Hz publishing rate (5ms timesteps)
- Drift accumulates over time (expect ~1cm/m error)

### Critical ROS2 Topics

| Topic | Type | Frequency | Producer | Consumer | Purpose |
|-------|------|-----------|----------|----------|---------|
| `/cmd_vel` | geometry_msgs/Twist | Variable | test nodes / strategy | odometry, Gazebo | Velocity commands |
| `/odom` | nav_msgs/Odometry | 200 Hz | inertial_odometry | RViz, planning, strategy | Pose estimate |
| `/camera` | sensor_msgs/Image | ~10 Hz | Gazebo bridge | nut_identifier2 | RGB camera feed |
| `/nut_detections` | nut_identifier_msgs/Nut | ~10 Hz | nut_identifier2 | strategy layer (mam_python) | Detected nuts |
| `/tf` | tf2_msgs/TFMessage | 200 Hz | inertial_odometry | TF listeners, RViz | Transform tree (odom→base_link) |

---

## Development Workflow

### Adding a New Feature

1. **Create feature branch** from appropriate base:
   ```bash
   git checkout dev
   git checkout -b feat/my-feature dev
   ```

2. **Implement & test locally**:
   ```bash
   colcon build --symlink-install
   source install/setup.bash
   colcon test
   ```

3. **Push & open PR**:
   ```bash
   git push origin feat/my-feature
   # → Open PR on GitHub
   ```

4. **CI automatically runs** (Docker build + tests)

5. **Merge to `dev`** after code review

6. **Later: cherry-pick to `main`** if stable

### Porting Strategy from `main` to `dev`

1. Prototype on `main` branch (use `ai_decision_demo.py` or `.cpp`)
2. Validate behavior with visualizer
3. Create feature branch: `feat/ros-strategy-integration`
4. Copy Python logic to `mam_python/decision/strategy_engine.py`
5. Create ROS2 wrapper in `mam_python/ros2_node.py`
6. Subscribe to `/odom` + `/nut_detections`
7. Publish to `/cmd_vel`
8. Test in closed loop: `ros2 launch mam_eurobot_2026 arena.launch.py`
9. PR → `dev` → `main`

### Running Tests

```bash
# All tests
colcon test

# Specific test file
pytest src/squirrelcup/mam_eurobot_2026/test/test_odometry.py -v

# With Docker (mimics CI)
docker build -t mam-ci -f .devcontainer/Dockerfile .devcontainer
docker run --rm -v "$PWD":/work mam-ci \
  bash -c "colcon build && source install/setup.bash && colcon test"
```

---

## Setup Instructions

### Minimal Setup (for `main` branch)
```bash
sudo apt-get install libsfml-dev
pip install matplotlib numpy opencv-python
```

### Complete Setup (for `dev` + `feat_*`)
```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/eurobot_2026_ws/src
cd ~/eurobot_2026_ws/src
git clone https://github.com/MAM-org/squirrelcup.git
cd ..
rosdep update
rosdep install -y --from-paths src --ignore-src
colcon build --symlink-install
```

### Docker (Recommended)
```bash
docker build -t mam-dev -f .devcontainer/Dockerfile .devcontainer
docker run -it --rm -v "$PWD":/work mam-dev bash
```

---

## CI/CD Pipeline

**GitHub Actions** (`.github/workflows/ci.yml`):
- **Triggers**: Push to `main`, `dev`, or any `feat_*` branch
- **Steps**:
  1. Build Docker image (ROS2 Humble + tools)
  2. `rosdep install` dependencies
  3. `colcon build --symlink-install`
  4. `colcon test` (pytest + launch_testing)
  5. Upload logs on failure
- **Parallel workers**: 4 (for multi-core builds)

Status badge: ![CI](https://github.com/MAM-org/squirrelcup/actions/workflows/ci.yml/badge.svg)

---

## Branch Status Summary

| Branch | Purpose | Maturity | Last Updated | For Whom |
|--------|---------|----------|--------------|----------|
| `main` | Algorithm research | 🟢 Stable | Oct 2025 | Researchers, algorithm designers |
| `dev` | ROS2 integration | 🟢 Stable | Oct 2025 | ROS2 engineers, system integrators |
| `feat_ros_python_core_integration` | Python decision core | 🟡 Beta | Jan 2026 | Decision logic developers |
| `feat_trajectory_planner` | Advanced planning | 🟡 WIP | – | Motion planning researchers |
| `feat_model_3` | Mechanical refinement | 🟡 WIP | – | Mechanical engineers |
| `nicolas_python_core` | Personal sandbox | 🔴 Experimental | – | Nicolas (individual work) |

---

## Key Contacts

- **Repository**: https://github.com/MAM-org/squirrelcup
- **Team**: MAM (Mechanical & Autonomous Mobile)
- **Contact**: liendomf@univ-smb.fr
- **Issues**: GitHub Issues tracker

---

**README Version 1.0** | January 2026
