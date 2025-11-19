"""Optimisation des points."""

from dataclasses import dataclass
from typing import List


@dataclass
class CrateValue:
    """Valeur d'une caisse."""
    crate_id: int
    base_points: int
    position_bonus: float  # multiplicateur selon position
    time_bonus: float  # multiplicateur selon temps restant
    
    def get_total_value(self) -> int:
        """Calcule la valeur totale."""
        return int(self.base_points * self.position_bonus * self.time_bonus)


class ScoreOptimizer:
    """Optimise les points collectés."""
    
    def __init__(self):
        """Initialise l'optimiseur."""
        self.crate_values: List[CrateValue] = []
        
    def add_crate_value(self, crate_id: int, base_points: int = 100):
        """Ajoute une valeur de caisse."""
        self.crate_values.append(CrateValue(crate_id, base_points, 1.0, 1.0))
    
    def update_bonuses(self, time_remaining: float, total_time: float):
        """Met à jour les bonus en fonction du temps."""
        for crate in self.crate_values:
            # Plus le temps est court, plus les bonus augmentent
            time_factor = 1.0 + (1.0 - time_remaining / total_time) * 0.5
            crate.time_bonus = time_factor
    
    def get_optimal_collection_order(self) -> List[int]:
        """Retourne l'ordre optimal de collecte."""
        sorted_crates = sorted(
            self.crate_values, 
            key=lambda c: -c.get_total_value()
        )
        return [c.crate_id for c in sorted_crates]
