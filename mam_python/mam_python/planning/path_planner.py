"""Planification de chemins optimaux."""

from typing import List, Tuple
from dataclasses import dataclass
import math


@dataclass
class Waypoint:
    """Point de passage."""
    x: float
    y: float
    theta: float = 0.0


class PathPlanner:
    """Planificateur de chemin."""
    
    def __init__(self):
        """Initialise le planificateur."""
        self.path: List[Waypoint] = []
        
    def plan_line(self, start_x: float, start_y: float, end_x: float, end_y: float, 
                  num_points: int = 10) -> List[Waypoint]:
        """Planifie un chemin linéaire.
        
        Args:
            start_x, start_y: Position de départ
            end_x, end_y: Position de fin
            num_points: Nombre de points intermédiaires
            
        Returns:
            Liste de waypoints
        """
        waypoints = []
        
        for i in range(num_points + 1):
            t = i / num_points
            x = start_x + (end_x - start_x) * t
            y = start_y + (end_y - start_y) * t
            
            # Calcul de l'angle
            if i == num_points:
                theta = math.atan2(end_y - start_y, end_x - start_x)
            else:
                next_t = min(1.0, (i + 1) / num_points)
                next_x = start_x + (end_x - start_x) * next_t
                next_y = start_y + (end_y - start_y) * next_t
                theta = math.atan2(next_y - y, next_x - x)
            
            waypoints.append(Waypoint(x, y, theta))
        
        self.path = waypoints
        return waypoints
    
    def plan_arc(self, center_x: float, center_y: float, radius: float, 
                 start_angle: float, end_angle: float, num_points: int = 20) -> List[Waypoint]:
        """Planifie un arc de cercle."""
        waypoints = []
        
        for i in range(num_points + 1):
            t = i / num_points
            angle = start_angle + (end_angle - start_angle) * t
            
            x = center_x + radius * math.cos(angle)
            y = center_y + radius * math.sin(angle)
            theta = angle + math.pi / 2
            
            waypoints.append(Waypoint(x, y, theta))
        
        self.path = waypoints
        return waypoints
    
    def get_path(self) -> List[Waypoint]:
        """Retourne le chemin actuel."""
        return self.path
