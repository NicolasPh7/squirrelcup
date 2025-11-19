"""Contrôle du bras robotique et de la pince."""

from enum import Enum
from dataclasses import dataclass


class GripperState(Enum):
    """État de la pince."""
    OPEN = 0
    CLOSED = 1
    MOVING = 2


@dataclass
class ArmCommand:
    """Commande pour le bras."""
    gripper_command: GripperState
    arm_angle: float = 0.0  # angle en degrés
    grip_force: float = 100.0  # force de serrage (0-100)


class ArmController:
    """Contrôle du bras robotique."""
    
    def __init__(self):
        """Initialise le contrôleur du bras."""
        self.gripper_state = GripperState.OPEN
        self.arm_angle = 0.0
        self.gripper_force = 50.0
        
    def open_gripper(self) -> ArmCommand:
        """Ouvre la pince."""
        self.gripper_state = GripperState.OPEN
        return ArmCommand(GripperState.OPEN, self.arm_angle)
    
    def close_gripper(self, force: float = 100.0) -> ArmCommand:
        """Ferme la pince.
        
        Args:
            force: Force de serrage (0-100)
        """
        self.gripper_state = GripperState.CLOSED
        self.gripper_force = min(force, 100.0)
        return ArmCommand(GripperState.CLOSED, self.arm_angle, self.gripper_force)
    
    def set_arm_angle(self, angle: float) -> ArmCommand:
        """Positionne le bras à un angle.
        
        Args:
            angle: Angle en degrés (0-180)
        """
        self.arm_angle = max(0, min(angle, 180))
        return ArmCommand(self.gripper_state, self.arm_angle, self.gripper_force)
    
    def grab_crate(self) -> ArmCommand:
        """Saisit une caisse."""
        return self.close_gripper(force=100.0)
    
    def release_crate(self) -> ArmCommand:
        """Relâche une caisse."""
        return self.open_gripper()
