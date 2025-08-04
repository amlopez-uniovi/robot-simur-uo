"""
Módulo de ejemplos de uso.
"""

# Importar las clases de robots simulados desde basic_navigation
from .basic_navigation import SimulatedDifferentialRobot, SimulatedAckermannRobot

# Los ejemplos como bug0_navigation_example y waypoints_creation_examples 
# se ejecutan directamente, no se importan

__all__ = [
    'SimulatedDifferentialRobot',  # Ejemplo de implementación diferencial
    'SimulatedAckermannRobot'      # Ejemplo de implementación Ackermann
]
]
