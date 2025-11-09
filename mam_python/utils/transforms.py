"""Utilitaires pour les transformations 3D et les coordonnées."""

import math
import numpy as np
from dataclasses import dataclass


@dataclass
class Transform3D:
    """Représente une transformation 3D (position + rotation)."""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    rx: float = 0.0  # rotation x (roll)
    ry: float = 0.0  # rotation y (pitch)
    rz: float = 0.0  # rotation z (yaw)


class TransformUtils:
    """Utilitaires pour les transformations."""
    
    @staticmethod
    def quaternion_to_euler(qx: float, qy: float, qz: float, qw: float) -> Tuple[float, float, float]:
        """Convertit un quaternion en angles d'Euler (roll, pitch, yaw)."""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (qw * qx + qy * qz)
        cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (qw * qy - qz * qx)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return (roll, pitch, yaw)
    
    @staticmethod
    def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
        """Convertit les angles d'Euler en quaternion (qx, qy, qz, qw)."""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        
        return (qx, qy, qz, qw)
