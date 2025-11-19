"""Moteur de stratégie décisionnelle."""

from enum import Enum
from dataclasses import dataclass
from typing import List, Callable


class StrategyMode(Enum):
    """Mode stratégique."""
    AGGRESSIVE = 0
    BALANCED = 1
    CONSERVATIVE = 2
    RETURN_HOME = 3


@dataclass
class StrategyDecision:
    """Décision stratégique."""
    action: str
    confidence: float
    reason: str


class StrategyEngine:
    """Moteur décisionnaire."""
    
    def __init__(self):
        """Initialise le moteur."""
        self.mode = StrategyMode.BALANCED
        self.time_remaining = 90.0
        
    def set_mode(self, mode: StrategyMode):
        """Change le mode stratégique."""
        self.mode = mode
    
    def update_time(self, elapsed_time: float, total_time: float):
        """Met à jour le temps restant."""
        self.time_remaining = total_time - elapsed_time
        
        # Changement automatique de stratégie
        if self.time_remaining < 10:
            self.mode = StrategyMode.RETURN_HOME
        elif self.time_remaining < total_time * 0.3:
            self.mode = StrategyMode.CONSERVATIVE
    
    def decide_next_action(self, robot_position: tuple, 
                          nearby_targets: List, 
                          battery_level: float) -> StrategyDecision:
        """Décide de la prochaine action."""
        
        # Si batterie faible, retour à la base
        if battery_level < 20:
            return StrategyDecision("return_home", 1.0, "Batterie faible")
        
        # Si temps faible, retour à la base
        if self.time_remaining < 10:
            return StrategyDecision("return_home", 1.0, "Temps faible")
        
        # Sinon, collecte les cibles
        if nearby_targets:
            return StrategyDecision("grab_nearest", 0.8, "Cibles disponibles")
        
        return StrategyDecision("explore", 0.5, "Exploration")
