"""
Módulo webots para robot_simur_uo

Este módulo contiene las clases para interactuar con robots en Webots:
- WebotsBaseDifferentialRobot: Clase base con funcionalidades comunes para robots diferenciales
- EPuck: Implementación específica para robot E-puck
- RosBot: Implementación específica para robot RosBot
"""

from .webots_base_differential_robot import WebotsBaseDifferentialRobot
from .epuck_robot import EPuck
from .rosbot_robot import RosBot

__all__ = ['WebotsBaseDifferentialRobot', 'EPuck', 'RosBot']
