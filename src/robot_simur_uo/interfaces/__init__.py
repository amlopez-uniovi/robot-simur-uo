"""
Módulo de interfaces para robots.
"""

from .irobot import IRobot, IRobotBase, IDifferentialRobot, IAckermannRobot

__all__ = [
    'IRobot',           # Alias para compatibilidad hacia atrás
    'IRobotBase',       # Interfaz base para todos los robots
    'IDifferentialRobot',  # Interfaz para robots diferenciales
    'IAckermannRobot'   # Interfaz para robots tipo Ackermann
]
