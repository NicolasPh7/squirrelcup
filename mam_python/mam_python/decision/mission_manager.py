"""Gestion des missions du robot."""

from enum import Enum
from dataclasses import dataclass, field
from typing import List
import time


class MissionState(Enum):
    """État d'une mission."""
    PENDING = 0
    ACTIVE = 1
    COMPLETED = 2
    FAILED = 3


@dataclass
class Mission:
    """Représente une mission."""
    id: int
    name: str
    priority: int = 0
    state: MissionState = MissionState.PENDING
    start_time: float = field(default_factory=time.time)
    estimated_duration: float = 30.0


class MissionManager:
    """Gère les missions du robot."""
    
    def __init__(self):
        """Initialise le gestionnaire de missions."""
        self.missions: List[Mission] = []
        self.current_mission = None
        self.next_mission_id = 0
        
    def create_mission(self, name: str, priority: int = 0, 
                      estimated_duration: float = 30.0) -> int:
        """Crée une nouvelle mission.
        
        Returns:
            ID de la mission
        """
        mission_id = self.next_mission_id
        self.next_mission_id += 1
        
        mission = Mission(mission_id, name, priority, 
                         estimated_duration=estimated_duration)
        self.missions.append(mission)
        return mission_id
    
    def start_mission(self, mission_id: int) -> bool:
        """Démarre une mission."""
        for mission in self.missions:
            if mission.id == mission_id:
                mission.state = MissionState.ACTIVE
                self.current_mission = mission
                return True
        return False
    
    def complete_mission(self, mission_id: int) -> bool:
        """Marque une mission comme complétée."""
        for mission in self.missions:
            if mission.id == mission_id:
                mission.state = MissionState.COMPLETED
                return True
        return False
    
    def get_next_mission(self) -> Mission or None:
        """Retourne la prochaine mission prioritaire."""
        pending = [m for m in self.missions if m.state == MissionState.PENDING]
        if pending:
            return sorted(pending, key=lambda m: -m.priority)[0]
        return None
