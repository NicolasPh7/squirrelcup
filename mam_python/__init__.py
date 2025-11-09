"""Architecture Python complète pour Eurobot 2026 - Squirrel Cup.

Structure modulaire:
- utils/: Utilitaires (logger, géométrie, transformations)
- core/: État du jeu et du robot
- control/: Contrôle des actionneurs
- perception/: Perception sensorielle
- planning/: Planification de trajectoires
- decision/: Prise de décision
- scoring/: Système de scoring
- tests/: Tests unitaires
"""

__version__ = "1.0.0"
__author__ = "Nicolas"

from .core import GameState, RobotState
from .control import MotionController, ArmController
from .perception import MapBuilder, ObjectManager, TargetSelector
from .planning import PathPlanner, CollisionChecker, TrajectoryGenerator
from .decision import ActionPlanner, MissionManager, StrategyEngine
from .scoring import ScoreTracker, ScorePredictor, ScoreOptimizer
from .utils import Logger, GeometryUtils, TransformUtils

__all__ = [
    'GameState', 'RobotState',
    'MotionController', 'ArmController',
    'MapBuilder', 'ObjectManager', 'TargetSelector',
    'PathPlanner', 'CollisionChecker', 'TrajectoryGenerator',
    'ActionPlanner', 'MissionManager', 'StrategyEngine',
    'ScoreTracker', 'ScorePredictor', 'ScoreOptimizer',
    'Logger', 'GeometryUtils', 'TransformUtils',
]
