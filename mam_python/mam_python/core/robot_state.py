"""Gestion de l'état du robot."""

from dataclasses import dataclass, field
from typing import Tuple
from enum import Enum
import time


class RobotMode(Enum):
    """Modes de fonctionnement du robot."""
    IDLE = 0
    MOVING = 1
    COLLECTING = 2
    RETURNING = 3
    ERROR = 4


@dataclass
class Position:
    """Position du robot."""
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0  # angle en radians
    timestamp: float = field(default_factory=time.time)


@dataclass
class RobotConfig:
    """Configuration du robot."""
    max_speed: float = 1.0  # m/s
    max_angular_speed: float = 1.0  # rad/s
    gripper_height: float = 0.3  # m
    gripper_force: float = 100.0  # N


class RobotState:
    """Gère l'état du robot."""
    
    def __init__(self, config: RobotConfig = None):
        """Initialise l'état du robot.
        
        Args:
            config: Configuration du robot
        """
        self.config = config or RobotConfig()
        self.position = Position()
        self.mode = RobotMode.IDLE
        
        # Batterie
        self.battery_level = 100.0  # %
        self.battery_drain_rate = 0.1  # % par seconde
        
        # Bras robotique
        self.gripper_open = True
        self.arm_angle = 0.0
        
        # Capteurs
        self.lidar_active = False
        self.camera_active = False
        
    def update_position(self, x: float, y: float, theta: float):
        """Met à jour la position du robot."""
        self.position = Position(x, y, theta)
    
    def get_position(self) -> Position:
        """Retourne la position actuelle."""
        return self.position
    
    def set_mode(self, mode: RobotMode):
        """Change le mode du robot."""
        self.mode = mode
    
    def update_battery(self, delta_time: float):
        """Met à jour le niveau de batterie."""
        self.battery_level -= self.battery_drain_rate * delta_time
        self.battery_level = max(0, self.battery_level)
    
    def grip(self):
        """Ferme la pince."""
        self.gripper_open = False
    
    def release(self):
        """Ouvre la pince."""
        self.gripper_open = True
    
    def is_battery_low(self, threshold: float = 20.0) -> bool:
        """Vérifie si la batterie est faible."""
        return self.battery_level < threshold
