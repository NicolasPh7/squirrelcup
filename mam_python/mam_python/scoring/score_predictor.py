"""Prédiction du score final."""

from typing import List, Tuple
import math


class ScorePredictor:
    """Prédit le score final possible."""
    
    def __init__(self):
        """Initialise le prédicteur."""
        self.crates_total = 20
        self.points_per_crate = 100
        
    def predict_final_score(self, collected_crates: int, 
                           time_remaining: float, 
                           total_time: float) -> Tuple[int, int]:
        """Prédit le score final.
        
        Returns:
            (score_conservatif, score_optimiste)
        """
        # Score conservatif : on suppose qu'on ne collecte plus rien
        conservative = collected_crates * self.points_per_crate
        
        # Score optimiste : on suppose une collecte constante
        if total_time > 0:
            collection_rate = collected_crates / (total_time - time_remaining + 1)
            remaining_collectable = collection_rate * time_remaining
            optimistic = int(collected_crates * self.points_per_crate + 
                            remaining_collectable * self.points_per_crate)
        else:
            optimistic = conservative
        
        return (conservative, optimistic)
    
    def get_best_score_strategy(self, available_crates: int, 
                               time_remaining: float) -> str:
        """Suggère la meilleure stratégie."""
        if time_remaining < 10:
            return "return_home"
        elif available_crates < 3:
            return "explore"
        else:
            return "collect"
