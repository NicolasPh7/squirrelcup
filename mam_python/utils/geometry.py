"""Utilitaires de géométrie pour les calculs de trajectoire."""

import math
from dataclasses import dataclass
from typing import Tuple, List


@dataclass
class Point:
    """Représente un point 2D."""
    x: float
    y: float
    
    def distance_to(self, other: 'Point') -> float:
        """Calcule la distance à un autre point."""
        return math.sqrt((self.x - other.x)**2 + (self.y - other.y)**2)
    
    def angle_to(self, other: 'Point') -> float:
        """Calcule l'angle vers un autre point (en radians)."""
        return math.atan2(other.y - self.y, other.x - self.x)


@dataclass
class Vector2D:
    """Représente un vecteur 2D."""
    x: float
    y: float
    
    def magnitude(self) -> float:
        """Retourne la magnitude du vecteur."""
        return math.sqrt(self.x**2 + self.y**2)
    
    def normalize(self) -> 'Vector2D':
        """Retourne le vecteur normalisé."""
        mag = self.magnitude()
        if mag == 0:
            return Vector2D(0, 0)
        return Vector2D(self.x / mag, self.y / mag)
    
    def dot(self, other: 'Vector2D') -> float:
        """Produit scalaire."""
        return self.x * other.x + self.y * other.y


class GeometryUtils:
    """Utilitaires de géométrie."""
    
    @staticmethod
    def line_intersection(p1: Point, p2: Point, p3: Point, p4: Point) -> Tuple[float, float] or None:
        """Calcule l'intersection de deux lignes."""
        x1, y1 = p1.x, p1.y
        x2, y2 = p2.x, p2.y
        x3, y3 = p3.x, p3.y
        x4, y4 = p4.x, p4.y
        
        denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
        if abs(denom) < 1e-10:
            return None
        
        t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom
        x = x1 + t * (x2 - x1)
        y = y1 + t * (y2 - y1)
        
        return (x, y)
    
    @staticmethod
    def point_in_circle(point: Point, center: Point, radius: float) -> bool:
        """Vérifie si un point est dans un cercle."""
        return point.distance_to(center) <= radius
    
    @staticmethod
    def angle_difference(a1: float, a2: float) -> float:
        """Calcule la différence d'angle normalisée [-π, π]."""
        diff = a2 - a1
        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi
        return diff
