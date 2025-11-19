"""Gestion et suivi des objets détectés."""

from dataclasses import dataclass, field
from typing import List, Dict
from enum import Enum
import time


class ObjectType(Enum):
    """Types d'objets détectables."""
    CRATE = 0
    OBSTACLE = 1
    ROBOT = 2
    WALL = 3


@dataclass
class DetectedObject:
    """Objet détecté par les capteurs."""
    id: int
    object_type: ObjectType
    x: float
    y: float
    confidence: float = 0.0
    last_seen: float = field(default_factory=time.time)
    track_id: int = -1


class ObjectManager:
    """Gère les objets détectés."""
    
    def __init__(self):
        """Initialise le gestionnaire d'objets."""
        self.objects: Dict[int, DetectedObject] = {}
        self.next_id = 0
        self.max_age = 5.0  # secondes avant suppression
        
    def add_object(self, obj_type: ObjectType, x: float, y: float, confidence: float = 0.8) -> int:
        """Ajoute un objet détecté."""
        obj_id = self.next_id
        self.next_id += 1
        
        self.objects[obj_id] = DetectedObject(
            obj_id, obj_type, x, y, confidence
        )
        return obj_id
    
    def update_object(self, obj_id: int, x: float, y: float, confidence: float = 0.8) -> bool:
        """Met à jour un objet détecté."""
        if obj_id in self.objects:
            self.objects[obj_id].x = x
            self.objects[obj_id].y = y
            self.objects[obj_id].confidence = confidence
            self.objects[obj_id].last_seen = time.time()
            return True
        return False
    
    def get_objects_by_type(self, obj_type: ObjectType) -> List[DetectedObject]:
        """Retourne tous les objets d'un type donné."""
        return [obj for obj in self.objects.values() if obj.object_type == obj_type]
    
    def get_nearest_object(self, x: float, y: float) -> DetectedObject or None:
        """Retourne l'objet le plus proche."""
        if not self.objects:
            return None
        
        nearest = min(
            self.objects.values(),
            key=lambda obj: (obj.x - x)**2 + (obj.y - y)**2
        )
        return nearest
    
    def clean_old_objects(self):
        """Supprime les objets trop vieux."""
        current_time = time.time()
        to_remove = [
            obj_id for obj_id, obj in self.objects.items()
            if current_time - obj.last_seen > self.max_age
        ]
        for obj_id in to_remove:
            del self.objects[obj_id]
