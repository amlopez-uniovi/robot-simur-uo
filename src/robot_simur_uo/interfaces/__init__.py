"""
Módulo de interfaces para robots.

Incluye interfaces base, diferencial y Ackermann.
"""

from .idifferential_robot import IDifferentialRobot
from .iackermann_robot import IAckermannRobot

__all__ = [
    'IDifferentialRobot',  # Interfaz para robots diferenciales
    'IAckermannRobot',  # Interfaz para robots tipo Ackermann
]
