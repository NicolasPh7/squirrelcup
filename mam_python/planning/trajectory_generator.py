"""Génération de trajectoires lissées."""

from typing import List
from dataclasses import dataclass
import math


@dataclass
class TrajectoryPoint:
    """Point de trajectoire avec commande."""
    x: float
    y: float
    theta: float
    v: float  # vitesse linéaire
    w: float  # vitesse angulaire


class TrajectoryGenerator:
    """Génère des trajectoires."""
    
    def __init__(self, dt: float = 0.1):
        """Initialise le générateur.
        
        Args:
            dt: Pas de temps en secondes
        """
        self.dt = dt
        
    def generate_trajectory(self, waypoints: List, max_speed: float = 1.0, 
                           max_angular_speed: float = 1.0) -> List[TrajectoryPoint]:
        """Génère une trajectoire lissée à partir de waypoints."""
        trajectory = []
        
        for i in range(len(waypoints) - 1):
            current = waypoints[i]
            next_wp = waypoints[i + 1]
            
            # Distance à parcourir
            dist = math.sqrt((next_wp.x - current.x)**2 + (next_wp.y - current.y)**2)
            
            # Angle cible
            target_angle = math.atan2(next_wp.y - current.y, next_wp.x - current.x)
            
            # Calcul des vitesses
            v = min(max_speed, dist / self.dt)
            angle_diff = target_angle - current.theta
            # Normaliser l'angle
            while angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            while angle_diff < -math.pi:
                angle_diff += 2 * math.pi
            
            w = min(max_angular_speed, angle_diff / self.dt)
            
            trajectory.append(TrajectoryPoint(
                current.x, current.y, current.theta, v, w
            ))
        
        return trajectory
