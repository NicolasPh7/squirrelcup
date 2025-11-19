"""Gestion de l'état du jeu pour Eurobot 2026."""

from dataclasses import dataclass, field
from typing import List, Dict
from enum import Enum
import time


class GamePhase(Enum):
    """Phases du jeu."""
    INITIALIZATION = 0
    STARTING = 1
    RUNNING = 2
    PAUSED = 3
    ENDED = 4


@dataclass
class Crate:
    """Représente une caisse de noisettes."""
    id: int
    x: float
    y: float
    collected: bool = False
    timestamp: float = field(default_factory=time.time)


class GameState:
    """Gère l'état global du jeu."""
    
    def __init__(self, match_duration: int = 90):
        """Initialise l'état du jeu.
        
        Args:
            match_duration: Durée du match en secondes (par défaut 90s)
        """
        self.match_duration = match_duration
        self.start_time = None
        self.phase = GamePhase.INITIALIZATION
        
        # Caisses disponibles
        self.crates: Dict[int, Crate] = {}
        self.collected_crates: List[int] = []
        
        # Score
        self.score = 0
        
    def start_match(self):
        """Démarre le match."""
        self.start_time = time.time()
        self.phase = GamePhase.RUNNING
    
    def get_elapsed_time(self) -> float:
        """Retourne le temps écoulé en secondes."""
        if self.start_time is None:
            return 0
        return time.time() - self.start_time
    
    def is_match_over(self) -> bool:
        """Vérifie si le match est terminé."""
        return self.get_elapsed_time() >= self.match_duration
    
    def add_crate(self, crate_id: int, x: float, y: float):
        """Ajoute une caisse au jeu."""
        self.crates[crate_id] = Crate(crate_id, x, y)
    
    def collect_crate(self, crate_id: int) -> bool:
        """Marque une caisse comme collectée."""
        if crate_id in self.crates and not self.crates[crate_id].collected:
            self.crates[crate_id].collected = True
            self.collected_crates.append(crate_id)
            return True
        return False
    
    def get_remaining_crates(self) -> List[Crate]:
        """Retourne les caisses non collectées."""
        return [c for c in self.crates.values() if not c.collected]
    
    def update_score(self, points: int):
        """Met à jour le score."""
        self.score += points
