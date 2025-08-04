"""
Controlador de navegación Bug0 simple.

Algoritmo simple:
1. Ir hacia el objetivo
2. Si hay obstáculo frontal, girar a la izquierda
3. Cuando no hay obstáculo, volver a ir al objetivo
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


# Ejemplo de uso simple
if __name__ == "__main__":
    print("🧪 PRUEBA SIMPLE DEL CONTROLADOR BUG0")
    
    controller = Bug0NavigationController()
    controller.set_target(2.0, 2.0)
    
    # Simular movimiento
    x, y, angle = 0.0, 0.0, 0.0
    
    for i in range(10):
        # Simular obstáculo en iteración 5
        sensors = [0.2] if i == 5 else [1.0]  # obstáculo / sin obstáculo
        
        drive, steer = controller.calculate_control_commands(x, y, angle, sensors)
        print(f"Paso {i+1}: pos=({x:.1f},{y:.1f}) -> vel={drive:.2f}, giro={steer:.2f}")
        
        # Simular movimiento
        x += drive * 0.1
        y += drive * 0.1
        angle += steer * 0.1
