"""Suivi des points en temps réel."""

from dataclasses import dataclass, field
from typing import Dict, List
import time


@dataclass
class ScoringEvent:
    """Événement de scoring."""
    event_type: str
    points: int
    timestamp: float = field(default_factory=time.time)
    description: str = ""


class ScoreTracker:
    """Suit les points du robot."""
    
    def __init__(self):
        """Initialise le tracker."""
        self.total_score = 0
        self.events: List[ScoringEvent] = []
        self.score_multiplier = 1.0
        
    def add_points(self, points: int, event_type: str, description: str = "") -> int:
        """Ajoute des points.
        
        Returns:
            Points réellement ajoutés
        """
        actual_points = int(points * self.score_multiplier)
        self.total_score += actual_points
        
        event = ScoringEvent(event_type, actual_points, description=description)
        self.events.append(event)
        
        return actual_points
    
    def get_total_score(self) -> int:
        """Retourne le score total."""
        return self.total_score
    
    def get_score_history(self) -> List[ScoringEvent]:
        """Retourne l'historique de scoring."""
        return self.events
    
    def set_multiplier(self, multiplier: float):
        """Change le multiplicateur de points."""
        self.score_multiplier = multiplier
