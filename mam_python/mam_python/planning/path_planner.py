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
        self.current_point_index = 0
        
    def increment_current_point_index(self):
        self.current_point_index += 1

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
    

    def plan_A_Star(self, start: tuple[float, float], goal: tuple[float, float], map, occ_threshold: float = 0.5) -> list[tuple[float,float]] | None:
        """Plan path using A* with 8-connected neighbors and line-of-sight simplification."""
        
        self.current_point_index = 0

        resolution = map.get_resolution()
        grid_width = map.get_grid_width()
        grid_height = map.get_grid_height()
        grid = map.get_grid()

        goal_cluster = map.get_cluster_around(goal[0], goal[1])  # may be None

        def to_cell(p):
            return (int(p[0] / resolution), int(p[1] / resolution))

        start_cell = to_cell(start)
        goal_cell  = to_cell(goal)

        # Guard: ensure start/goal are inside grid
        for (cx, cy), name in ((start_cell, "start"), (goal_cell, "goal")):
            if not (0 <= cx < grid_width and 0 <= cy < grid_height):
                print(f"{name} cell out of bounds: {cx},{cy}")
                return None

        def heuristic(a, b):
            return math.hypot(a[0] - b[0], a[1] - b[1])

        neighbors8 = [(1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)]

        open_set: list[tuple[float, tuple[int,int]]] = [(0.0, start_cell)]
        came_from: dict[tuple[int,int], tuple[int,int]] = {}
        g_score = {start_cell: 0.0}
        visited = set()

        while open_set:
            _, current = heapq.heappop(open_set)
            if current in visited:
                continue
            visited.add(current)

            if current == goal_cell:
                # Reconstruct (cells -> meters)
                path_cells = [current]
                while current in came_from:
                    current = came_from[current]
                    path_cells.append(current)
                path_cells.reverse()
                path = [(c[0] * resolution, c[1] * resolution) for c in path_cells]

                # Line-of-sight simplification with threshold
                path = self.line_of_sight(path, grid, resolution, occ_threshold=occ_threshold, inflation_cells=1)
                self.path = self.to_waypoints_with_theta(path)
                return path

            cx, cy = current
            for dx, dy in neighbors8:
                nx, ny = cx + dx, cy + dy
                if not (0 <= nx < grid_width and 0 <= ny < grid_height):
                    continue

                occupied = grid[ny, nx] > occ_threshold
                # Allow “goal cluster” override only if the occupied cell is within that cluster
                if occupied:
                    if goal_cluster is None or not goal_cluster.contains_point(nx, ny):
                        continue

                step_cost = math.hypot(dx, dy)  # 1 for axial, sqrt(2) for diagonal
                tentative_g = g_score[current] + step_cost
                if tentative_g < g_score.get((nx, ny), float("inf")):
                    came_from[(nx, ny)] = current
                    g_score[(nx, ny)] = tentative_g
                    f_score = tentative_g + heuristic((nx, ny), goal_cell)
                    heapq.heappush(open_set, (f_score, (nx, ny)))

        return None

    
    def line_of_sight(self, path, grid, resolution, occ_threshold: float = 0.5, inflation_cells: int = 0):
        """Simplify path by skipping intermediate points if straight segments are free."""
        if not path:
            return []

        # Optional binary inflation for safety margin
        inflated = grid if inflation_cells <= 0 else self.inflate_grid_binary(grid, inflation_cells, occ_threshold)

        smoothed = [path[0]]
        i = 0
        n = len(path)
        while i < n - 1:
            j = n - 1
            while j > i + 1:
                if self.is_free_line(path[i], path[j], inflated, resolution, occ_threshold):
                    break
                j -= 1
            smoothed.append(path[j])
            i = j
        return smoothed

    def inflate_grid_binary(self, grid, r, occ_threshold=0.5):
        """Inflate obstacles in a binary manner around cells above threshold."""
        import numpy as np
        h, w = grid.shape
        inflated = grid.copy()
        if r <= 0:
            return inflated
        # Create a binary mask for obstacles
        mask = (grid > occ_threshold).astype(np.uint8)
        # Naive square inflation (replace with morphology if available)
        for y in range(h):
            for x in range(w):
                if mask[y, x]:
                    y0, y1 = max(0, y - r), min(h - 1, y + r)
                    x0, x1 = max(0, x - r), min(w - 1, x + r)
                    inflated[y0:y1+1, x0:x1+1] = 1.0
        return inflated


    def is_free_line(self, p1, p2, grid, resolution, occ_threshold: float = 0.5):
        """Bresenham line check using occupancy threshold and bounds."""
        h, w = grid.shape

        def clamp_cell(x, y):
            return 0 <= x < w and 0 <= y < h

        x1, y1 = int(p1[0] / resolution), int(p1[1] / resolution)
        x2, y2 = int(p2[0] / resolution), int(p2[1] / resolution)

        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        sx = 1 if x1 < x2 else -1
        sy = 1 if y1 < y2 else -1
        err = dx - dy

        while True:
            if not clamp_cell(x1, y1):
                return False
            if grid[y1, x1] > occ_threshold:
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
    
    def to_waypoints_with_theta(self, pts: list[tuple[float,float]]) -> List[Waypoint]:
        waypoints: List[Waypoint] = []
        if not pts:
            return waypoints
        n = len(pts)
        for i in range(n):
            x, y = pts[i]
            if i < n - 1:
                nx, ny = pts[i + 1]
                theta = math.atan2(ny - y, nx - x)
            else:
                px, py = pts[i - 1] if i > 0 else (x, y)
                theta = math.atan2(y - py, x - px)
            waypoints.append(Waypoint(x, y, theta))
        return waypoints
    
    def smooth_with_catmull_rom(self, pts: list[tuple[float,float]], samples_per_seg: int = 10,
                                kappa_max: float | None = None) -> list[tuple[float,float]]:
        if len(pts) < 2:
            return pts
        import numpy as np

        P = np.array(pts, dtype=float)
        # Duplicate endpoints for boundary tangents
        P_ext = np.vstack([P[0], P, P[-1]])

        def catmull_rom(p0, p1, p2, p3, t):
            # Standard Catmull–Rom (uniform) basis
            t2 = t*t
            t3 = t2*t
            return 0.5 * (
                (2*p1) +
                (-p0 + p2) * t +
                (2*p0 - 5*p1 + 4*p2 - p3) * t2 +
                (-p0 + 3*p1 - 3*p2 + p3) * t3
            )

        dense = []
        for i in range(1, len(P_ext)-2):
            p0, p1, p2, p3 = P_ext[i-1], P_ext[i], P_ext[i+1], P_ext[i+2]
            for s in range(samples_per_seg):
                t = s / float(samples_per_seg)
                xy = catmull_rom(p0, p1, p2, p3, t)
                dense.append((float(xy[0]), float(xy[1])))
        dense.append(tuple(P[-1]))

        # Optional curvature-based decimation (approximate finite differences)
        if kappa_max is not None and kappa_max > 0.0:
            decimated = [dense[0]]
            for i in range(2, len(dense)-2):
                xm, x0, x1, xp = dense[i-1][0], dense[i][0], dense[i+1][0], dense[i+2][0]
                ym, y0, y1, yp = dense[i-1][1], dense[i][1], dense[i+1][1], dense[i+2][1]
                dx = x1 - x0
                dy = y1 - y0
                ddx = xp - 2*x1 + x0
                ddy = yp - 2*y1 + y0
                num = abs(dx*ddy - dy*ddx)
                den = (dx*dx + dy*dy) ** 1.5 + 1e-9
                kappa = num / den
                if kappa <= kappa_max:
                    # Skip intermediate point; keep path smoother
                    continue
                decimated.append(dense[i])
            decimated.append(dense[-1])
            return decimated

        return dense

    def get_path(self) -> List[Waypoint]:
        """Retourne le chemin actuel."""
        return self.path
