"""Construction et gestion de la carte de l'environnement."""

import pdb

from dataclasses import dataclass, field
from typing import List, Tuple
import numpy as np
import math

from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point, Pose
from nav_msgs.msg import Path, OccupancyGrid

from mam_python.core import Position

@dataclass
class GridCell:
    """Cellule de la grille."""
    x: int
    y: int
    occupied: bool = False
    confidence: float = 0.0  # 0-1

@dataclass
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
    
    def get_width(self): 
        return self.width
    def get_height(self): 
        return self.height
    def get_resolution(self): 
        return self.resolution
    def get_grid_width(self): 
        return self.grid_width
    def get_grid_height(self): 
        return self.grid_height
    def get_grid(self): 
        return self.grid
    
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
    

    def update_from_marker_array(self, marker_array: MarkerArray, robot_pose: Position) -> bool:
        for marker in marker_array.markers:
            if marker.action == marker.DELETEALL:
                self.clear()
                continue

            # print(f"robot_pose= {robot_pose}")
            
            # Marker-Koordinaten im base_link Frame
            cx_bl = marker.pose.position.x
            cy_bl = marker.pose.position.y

            # print(f"marker.pose.position= {marker.pose.position}")


            # Transformation: base_link → map
            cos_t = math.cos(robot_pose.theta)
            sin_t = math.sin(robot_pose.theta)

            cx = robot_pose.x + cos_t * cx_bl - sin_t * cy_bl
            cy = robot_pose.y + sin_t * cx_bl + cos_t * cy_bl

            # print(f"Transformed marker.pose.position= {cx}, {cy}")


            # Ausdehnung (in Metern)
            sx = marker.scale.x
            sy = marker.scale.y

            # Rechteckgrenzen im map-Frame
            x_min = cx - sx / 2.0
            x_max = cx + sx / 2.0
            y_min = cy - sy / 2.0
            y_max = cy + sy / 2.0

            gx_min = int(x_min / self.resolution)
            gx_max = int(x_max / self.resolution)
            gy_min = int(y_min / self.resolution)
            gy_max = int(y_max / self.resolution)

            for gx in range(gx_min, gx_max + 1):
                for gy in range(gy_min, gy_max + 1):
                    if 0 <= gx < self.grid_width and 0 <= gy < self.grid_height:
                        self.grid[gy, gx] = 1.0

        return True
    
    def clear(self, value: float = 0.0):
        """
        Réinitialise toute la grille.
        
        Args:
            value: Valeur initiale pour chaque cellule (par défaut 0.0 = libre)
        """
        self.grid[:, :] = value

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

    def publish_occupancy_grid(self, stamp, occupancy_pub):
        msg = OccupancyGrid()
        msg.header.frame_id = "map"
        msg.header.stamp = stamp

        # Map info
        msg.info.resolution = self.resolution
        msg.info.width = self.grid_width
        msg.info.height = self.grid_height

        # pdb.set_trace()

        # Ursprung der Karte (unten links)
        origin = Pose()
        # Quaternion aus yaw = 3.1415  from mam_eurobot_2026/worlds/arena_world.sdf:66
        qz = math.sin(-math.pi)
        qw = math.cos(-math.pi)

        origin.position.x = 0.0 # from mam_eurobot_2026/worlds/arena_world.sdf:66
        origin.position.y = 0.0 # from mam_eurobot_2026/worlds/arena_world.sdf:66
        origin.position.z = 0.0
        origin.orientation.z = qz
        origin.orientation.w = qw
        msg.info.origin = origin

        # Grid in int8 konvertieren
        data = []
        for gy in range(self.grid_height):
            for gx in range(self.grid_width):
                val = self.grid[gy, gx]
                if val <= 0.0:
                    data.append(0)      # frei
                elif val >= 1.0:
                    data.append(100)    # sicher besetzt
                else:
                    data.append(int(val * 100))  # Wahrscheinlichkeit
        msg.data = data
        # pdb.set_trace()
        occupancy_pub.publish(msg)