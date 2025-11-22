"""Contrôle des moteurs de mouvement du robot."""

from dataclasses import dataclass
from typing import Tuple
import math


@dataclass
class MotorCommand:
    """Commande moteur."""
    left_speed: float  # vitesse roue gauche (-1 à 1)
    right_speed: float  # vitesse roue droite (-1 à 1)
    duration: float = 0.0  # durée en secondes


class MotionController:
    """Contrôle des moteurs de mouvement."""
    
    def __init__(self, max_speed: float = 1.0):
        """Initialise le contrôleur de mouvement.
        
        Args:
            max_speed: Vitesse maximale (0-1)
        """
        self.max_speed = max_speed
        self.left_speed = 0.0
        self.right_speed = 0.0
        
    def forward(self, speed: float = 1.0) -> MotorCommand:
        """Commande d'avance."""
        speed = min(abs(speed), self.max_speed)
        return MotorCommand(speed, speed)
    
    def backward(self, speed: float = 1.0) -> MotorCommand:
        """Commande de recul."""
        speed = min(abs(speed), self.max_speed)
        return MotorCommand(-speed, -speed)
    
    def turn_left(self, speed: float = 1.0) -> MotorCommand:
        """Tourne à gauche."""
        speed = min(abs(speed), self.max_speed)
        return MotorCommand(-speed, speed)
    
    def turn_right(self, speed: float = 1.0) -> MotorCommand:
        """Tourne à droite."""
        speed = min(abs(speed), self.max_speed)
        return MotorCommand(speed, -speed)
    
    def arc(self, angle: float, radius: float, speed: float = 1.0) -> MotorCommand:
        """Effectue un mouvement en arc de cercle."""
        speed = min(abs(speed), self.max_speed)
        
        # Calcul des vitesses pour un arc
        if radius == 0:
            return MotorCommand(0, 0)
        
        # Ratio entre les deux roues selon le rayon
        wheel_distance = 0.3  # distance entre les roues en mètres
        ratio = (radius - wheel_distance/2) / (radius + wheel_distance/2)
        
        left_speed = speed
        right_speed = speed * ratio
        
        return MotorCommand(left_speed, right_speed)
    
    def stop(self) -> MotorCommand:
        """Arrête le robot."""
        return MotorCommand(0, 0)
