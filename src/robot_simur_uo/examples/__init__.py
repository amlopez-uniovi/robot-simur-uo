"""
Módulo de ejemplos de uso.
"""

# Importar las clases de robots simulados desde basic_navigation
from .basic_navigation import SimulatedDifferentialRobot, SimulatedAckermannRobot

__all__ = [
    'SimulatedDifferentialRobot',  # Ejemplo de implementación diferencial
    'SimulatedAckermannRobot'      # Ejemplo de implementación Ackermann
]
