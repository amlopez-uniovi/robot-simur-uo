"""
Módulo webots para robot_simur_uo

Este módulo contiene las clases para interactuar con robots en Webots:
- BaseRobot: Clase base con funcionalidades comunes
- EPuck: Implementación específica para robot E-puck
- RosBot: Implementación específica para robot RosBot
"""

from .base_robot import BaseRobot
from .epuck_robot import EPuck
from .rosbot_robot import RosBot

__all__ = ['BaseRobot', 'EPuck', 'RosBot']
