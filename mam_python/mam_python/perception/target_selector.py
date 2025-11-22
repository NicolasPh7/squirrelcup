"""Sélection de la cible prioritaire pour la collecte."""

from dataclasses import dataclass
from typing import List, Tuple
import math


@dataclass
class Target:
    """Représente une cible de collecte."""
    id: int
    x: float
    y: float
    priority: float = 0.0
    score: float = 0.0


class TargetSelector:
    """Sélectionne la meilleure cible."""
    
    @staticmethod
    def distance_priority(targets: List[Target], robot_x: float, robot_y: float) -> Target | None:
        """Sélectionne la cible la plus proche."""
        if not targets:
            return None
        
        nearest = min(
            targets,
            key=lambda t: math.sqrt((t.x - robot_x)**2 + (t.y - robot_y)**2)
        )
        return nearest
    
    @staticmethod
    def score_priority(targets: List[Target]) -> Target | None:
        """Sélectionne la cible avec le meilleur score."""
        if not targets:
            return None
        
        return max(targets, key=lambda t: t.score)
    
    @staticmethod
    def time_priority(targets: List[Target], match_time: float, total_time: float) -> Target | None:
        """Sélectionne basé sur le temps restant."""
        if not targets:
            return None
        
        # Plus la time diminue, plus on favorise les cibles hautes valeur
        if match_time > total_time * 0.7:  # Début du match
            return TargetSelector.distance_priority(targets, 0, 0)
        else:  # Fin du match
            return TargetSelector.score_priority(targets)
