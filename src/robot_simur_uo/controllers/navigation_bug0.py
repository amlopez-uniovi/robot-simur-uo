"""
Controlador de navegación Bug0 simple.

Algoritmo simple:
    1. Ir hacia el objetivo
    2. Si hay obstáculo frontal, girar a la izquierda
    3. Cuando no hay obstáculo, volver a ir al objetivo

Ejemplo:
    >>> ctrl = Bug0NavigationController()
    >>> ctrl.set_target(1.0, 2.0)
    >>> v, w = ctrl.calculate_control_commands(0, 0, 0, front_distance=0.6)
    >>> print(v, w)
"""

import math
from typing import Tuple, List

try:
    from .navigation import NavigationController
except ImportError:
    import sys
    import os
    sys.path.append(os.path.dirname(os.path.abspath(__file__)))
    from navigation import NavigationController


class Bug0NavigationController(NavigationController):
    """
    Controlador Bug0 simple: ir al objetivo, si hay obstáculo girar a la izquierda.
    """
    
    def __init__(self, linear_gain: float = 1.0, steering_gain: float = 2.0, obstacle_threshold: float = 0.5):
        """Inicializa el controlador Bug0 simple."""
        super().__init__(linear_gain, steering_gain)
        self.obstacle_threshold = obstacle_threshold 
        
    def calculate_control_commands(self, 
                                 current_x: float, 
                                 current_y: float, 
                                 current_angle: float,
                                 front_distance: float = None) -> Tuple[float, float]:
        """
        Bug0 simple: ir al objetivo, si hay obstáculo girar izquierda.
        
        Args:
            current_x: Posición X actual
            current_y: Posición Y actual  
            current_angle: Ángulo actual
            front_distance: Distancia frontal (opcional)

        Returns:
            Tupla (velocidad_lineal, velocidad_angular)
        """
        # Si no hay sensores, usar navegación básica
        if front_distance is None:
            return super().calculate_control_commands(current_x, current_y, current_angle)
        
 
        if front_distance < self.obstacle_threshold:
            # Hay obstáculo: girar a la izquierda
            return 0.0, 1.0  # velocidad nula, girar izquierda
        else:
            # No hay obstáculo: ir al objetivo
            return super().calculate_control_commands(current_x, current_y, current_angle)

