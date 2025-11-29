"""Planification de chemins optimaux."""

from typing import List, Tuple
from dataclasses import dataclass
import math
import heapq
import math

from mam_python.perception import MapBuilder

@dataclass
class Waypoint:
    """Point de passage."""
    x: float
    y: float
    theta: float = 0.0


class PathPlanner:
    """Planificateur de chemin."""
    
    def __init__(self):
        """Initialise le planificateur."""
        self.path: List[Waypoint] = []
        
    def plan_line(self, start_x: float, start_y: float, end_x: float, end_y: float, 
                  num_points: int = 10) -> List[Waypoint]:
        """Planifie un chemin linéaire.
        
        Args:
            start_x, start_y: Position de départ
            end_x, end_y: Position de fin
            num_points: Nombre de points intermédiaires
            
        Returns:
            Liste de waypoints
        """
        waypoints = []
        
        for i in range(num_points + 1):
            t = i / num_points
            x = start_x + (end_x - start_x) * t
            y = start_y + (end_y - start_y) * t
            
            # Calcul de l'angle
            if i == num_points:
                theta = math.atan2(end_y - start_y, end_x - start_x)
            else:
                next_t = min(1.0, (i + 1) / num_points)
                next_x = start_x + (end_x - start_x) * next_t
                next_y = start_y + (end_y - start_y) * next_t
                theta = math.atan2(next_y - y, next_x - x)
            
            waypoints.append(Waypoint(x, y, theta))
        
        self.path = waypoints
        return waypoints
    
    def plan_arc(self, center_x: float, center_y: float, radius: float, 
                 start_angle: float, end_angle: float, num_points: int = 20) -> List[Waypoint]:
        """Planifie un arc de cercle."""
        waypoints = []
        
        for i in range(num_points + 1):
            t = i / num_points
            angle = start_angle + (end_angle - start_angle) * t
            
            x = center_x + radius * math.cos(angle)
            y = center_y + radius * math.sin(angle)
            theta = angle + math.pi / 2
            
            waypoints.append(Waypoint(x, y, theta))
        
        self.path = waypoints
        return waypoints
    

    def plan_A_Star(self, start: tuple[float, float], goal: tuple[float, float], map) -> list[tuple[float,float]] | None:
        """Plan path using the A* algorithm with line-of-sight smoothing"""
        resolution = map.get_resolution()
        grid_width = map.get_grid_width()
        grid_height = map.get_grid_height()
        grid = map.get_grid()

        not_obstacles = map.get_cluster_around(goal[0], goal[1])

        # Start- und Zielzellen
        start_cell = (int(start[0] / resolution), int(start[1] / resolution))
        goal_cell = (int(goal[0] / resolution), int(goal[1] / resolution))

        def heuristic(a, b):
            return math.hypot(a[0]-b[0], a[1]-b[1])

        open_set: list[tuple[float, tuple[int,int]]] = [(0.0, start_cell)]
        came_from = {}
        g_score = {start_cell: 0}

        while open_set:
            _, current = heapq.heappop(open_set)
            if current == goal_cell:
                # Rekonstruiere Pfad
                path = []
                while current in came_from:
                    x, y = current
                    path.append((x * resolution, y * resolution))
                    current = came_from[current]
                # path.append((goal_cell[0] * resolution, goal_cell[1] * resolution))
                path.reverse()
                # path = self.line_of_sight(path, grid, resolution)
                return path

            cx, cy = current
            for dx, dy in [(1,0), (-1,0), (0,1), (0,-1)]:
                neighbor = (cx+dx, cy+dy)
                nx, ny = neighbor
                if not (0 <= nx < grid_width and 0 <= ny < grid_height):
                    continue
                if grid[ny, nx] > 0.0 and not not_obstacles.contains_point(nx, ny):  # besetzt and not the target
                    continue

                tentative_g = g_score[current] + 1
                if tentative_g < g_score.get(neighbor, float("inf")):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = float(tentative_g) + heuristic(neighbor, goal_cell)
                    heapq.heappush(open_set, (f_score, neighbor))

        return None  # kein Pfad gefunden

    
    def line_of_sight(self, path, grid, resolution):
        smoothed = [path[0]]
        i = 0
        while i < len(path)-1:
            j = len(path)-1
            while j > i+1:
                if self.is_free_line(path[i], path[j], grid, resolution):
                    break
                j -= 1
            smoothed.append(path[j])
            i = j
        return smoothed


    def is_free_line(self, p1, p2, grid, resolution):
        x1, y1 = int(p1[0] / resolution), int(p1[1] / resolution)
        x2, y2 = int(p2[0] / resolution), int(p2[1] / resolution)

        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        sx = 1 if x1 < x2 else -1
        sy = 1 if y1 < y2 else -1
        err = dx - dy

        while True:
            if grid[y1, x1] > 0.0:  # Hindernis
                return False
            if x1 == x2 and y1 == y2:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x1 += sx
            if e2 < dx:
                err += dx
                y1 += sy
        return True
    def get_path(self) -> List[Waypoint]:
        """Retourne le chemin actuel."""
        return self.path
