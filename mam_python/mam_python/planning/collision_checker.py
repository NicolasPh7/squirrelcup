"""Vérification des collisions."""

from typing import List, Tuple
import math


class CollisionChecker:
    """Vérifie les collisions."""
    
    def __init__(self, robot_radius: float = 0.2):
        """Initialise le vérificateur.
        
        Args:
            robot_radius: Rayon du robot en mètres
        """
        self.robot_radius = robot_radius
        self.obstacles: List[Tuple[float, float, float]] = []  # (x, y, radius)
        
    def add_obstacle(self, x: float, y: float, radius: float = 0.1):
        """Ajoute un obstacle."""
        self.obstacles.append((x, y, radius))
    
    def check_collision(self, x: float, y: float) -> bool:
        """Vérifie si une position collisionne."""
        for obs_x, obs_y, obs_radius in self.obstacles:
            dist = math.sqrt((x - obs_x)**2 + (y - obs_y)**2)
            if dist < (self.robot_radius + obs_radius):
                return True
        return False
    
    def check_path_collision(self, path: List[Tuple[float, float]]) -> bool:
        """Vérifie si un chemin collisionne."""
        for x, y in path:
            if self.check_collision(x, y):
                return True
        return False
    
    def get_collision_free_point(self, x: float, y: float, search_radius: float = 1.0) -> Tuple[float, float] or None:
        """Trouve un point libre proche d'une position."""
        if not self.check_collision(x, y):
            return (x, y)
        
        # Recherche en spirale
        for r in [0.1 * i for i in range(1, int(search_radius / 0.1))]:
            for angle in [0.1 * i for i in range(0, 63)]:
                nx = x + r * math.cos(angle)
                ny = y + r * math.sin(angle)
                if not self.check_collision(nx, ny):
                    return (nx, ny)
        
        return None
