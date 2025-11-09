"""Planification des actions du robot."""

from enum import Enum
from dataclasses import dataclass
from typing import List, Callable


class ActionType(Enum):
    """Types d'actions."""
    MOVE_TO = 0
    GRAB = 1
    RELEASE = 2
    RETURN = 3
    WAIT = 4


@dataclass
class Action:
    """Représente une action."""
    action_type: ActionType
    parameters: dict = None
    priority: int = 0
    completed: bool = False


class ActionPlanner:
    """Planifie les actions du robot."""
    
    def __init__(self):
        """Initialise le planificateur d'actions."""
        self.actions: List[Action] = []
        self.current_action = None
        
    def add_action(self, action_type: ActionType, parameters: dict = None, priority: int = 0):
        """Ajoute une action à la queue."""
        action = Action(action_type, parameters or {}, priority, False)
        self.actions.append(action)
        self.actions.sort(key=lambda a: -a.priority)
        
    def get_next_action(self) -> Action or None:
        """Retourne la prochaine action à exécuter."""
        for action in self.actions:
            if not action.completed:
                self.current_action = action
                return action
        return None
    
    def complete_current_action(self):
        """Marque l'action actuelle comme complétée."""
        if self.current_action:
            self.current_action.completed = True
    
    def clear_completed_actions(self):
        """Supprime les actions complétées."""
        self.actions = [a for a in self.actions if not a.completed]
