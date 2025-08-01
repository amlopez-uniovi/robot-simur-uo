"""
Módulo de ejemplos de uso.
"""

# Solo exportar las clases de robots simulados por ahora
from .simulated_differential_robot import SimulatedDifferentialRobot
from .simulated_ackermann_robot import SimulatedAckermannRobot

__all__ = [
    'SimulatedDifferentialRobot',  # Ejemplo de implementación diferencial
    'SimulatedAckermannRobot'      # Ejemplo de implementación Ackermann
]
