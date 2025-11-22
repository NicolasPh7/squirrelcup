"""Construction et gestion de la carte de l'environnement."""

from dataclasses import dataclass, field
from typing import List, Tuple
import numpy as np


@dataclass
class GridCell:
    """Cellule de la grille."""
    x: int
    y: int
    occupied: bool = False
    confidence: float = 0.0  # 0-1


class MapBuilder:
    """Construiseur de carte pour la navigation."""
    
    def __init__(self, width: float = 3.0, height: float = 2.0, resolution: float = 0.05):
        """Initialise le constructeur de carte.
        
        Args:
            width: Largeur de la carte en mètres
            height: Hauteur de la carte en mètres
            resolution: Résolution de la grille en mètres
        """
        self.width = width
        self.height = height
        self.resolution = resolution
        
        self.grid_width = int(width / resolution)
        self.grid_height = int(height / resolution)
        self.grid = np.zeros((self.grid_height, self.grid_width))
        
    def update_cell(self, x: float, y: float, occupied: bool = True, confidence: float = 1.0):
        """Met à jour une cellule de la grille.
        
        Args:
            x, y: Coordonnées en mètres
            occupied: Si la cellule est occupée
            confidence: Niveau de confiance (0-1)
        """
        gx = int(x / self.resolution)
        gy = int(y / self.resolution)
        
        if 0 <= gx < self.grid_width and 0 <= gy < self.grid_height:
            if occupied:
                self.grid[gy, gx] = min(1.0, self.grid[gy, gx] + confidence)
            else:
                self.grid[gy, gx] = max(0.0, self.grid[gy, gx] - confidence)
    
    def is_occupied(self, x: float, y: float, threshold: float = 0.5) -> bool:
        """Vérifie si une position est occupée."""
        gx = int(x / self.resolution)
        gy = int(y / self.resolution)
        
        if 0 <= gx < self.grid_width and 0 <= gy < self.grid_height:
            return self.grid[gy, gx] > threshold
        return False
    
    def get_free_neighbors(self, x: float, y: float, distance: float = 0.1) -> List[Tuple[float, float]]:
        """Retourne les cellules libres autour d'une position."""
        neighbors = []
        for dx in [-distance, 0, distance]:
            for dy in [-distance, 0, distance]:
                if dx == 0 and dy == 0:
                    continue
                nx, ny = x + dx, y + dy
                if not self.is_occupied(nx, ny):
                    neighbors.append((nx, ny))
        return neighbors
