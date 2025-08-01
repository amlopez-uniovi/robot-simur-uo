"""
Módulo de interfaces para robots.
"""

from .irobot_base import IRobotBase
from .idifferential_robot import IDifferentialRobot
from .iackermann_robot import IAckermannRobot

__all__ = [
    'IRobotBase',       # Interfaz base para todos los robots
    'IDifferentialRobot',  # Interfaz para robots diferenciales
    'IAckermannRobot',  # Interfaz para robots tipo Ackermann
]
