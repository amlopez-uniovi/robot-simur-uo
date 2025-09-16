"""
Módulo webots para robot_simur_uo.

Contiene las clases para interactuar con robots en Webots:
	- WebotsDifferentialRobotLGC: Clase base para robots diferenciales con LiDAR, GPS y Compass
	- EPuck: Implementación específica para robot E-puck
	- RosBot: Implementación específica para robot RosBot
"""

from .webots_base_differential_robot import WebotsDifferentialRobotLGC
from .epuck_robot import EPuck
from .rosbot_robot import RosBot

__all__ = ['WebotsDifferentialRobotLGC', 'EPuck', 'RosBot']
