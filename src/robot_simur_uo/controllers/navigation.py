"""
Controlador de navegación básica para robots.
"""

import math
from typing import Tuple, Optional


class NavigationController:
    """Controlador para navegación básica de robots."""
    
    def __init__(self, max_speed: float = 1.0, linear_gain = 2.0, angular_gain = 2.0):
        """
        Inicializa el controlador de navegación.
        
        Args:
            max_speed: Velocidad máxima del robot
        """
        self.max_speed = max_speed
        self.target_x: Optional[float] = None
        self.target_y: Optional[float] = None
        self.linear_gain = linear_gain  # Ganancia lineal para velocidad
        self.angular_gain = angular_gain  # Ganancia angular para velocidad
        
    def set_target(self, x: float, y: float, tol: float = 0.1):
        """
        Establece el objetivo de navegación.
        
        Args:
            x: Coordenada X del objetivo
            y: Coordenada Y del objetivo
        """
        self.target_x = x
        self.target_y = y
        self.tolerance = tol
    
    def calculate_direction_to_target(self, current_x: float, current_y: float, 
                                    current_angle: float) -> Tuple[float, float]:
        """
        Calcula la dirección hacia el objetivo.
        
        Args:
            current_x: Posición X actual
            current_y: Posición Y actual
            current_angle: Ángulo actual del robot
            
        Returns:
            Tuple con (diferencia_angular, distancia_al_objetivo)
        """
        if self.target_x is None or self.target_y is None:
            return 0.0, 0.0
        
        # Calcular vector hacia el objetivo
        dx = self.target_x - current_x
        dy = self.target_y - current_y
        
        # Calcular ángulo hacia el objetivo
        target_angle = math.atan2(dy, dx)
        
        #np.arctan2(np.sin(th1 - th2), np.cos(th1 - th2))
        
        # Calcular diferencia angular
        angle_diff = math.atan2(math.sin(target_angle - current_angle), math.cos(target_angle - current_angle))
        
        # Calcular distancia
        distance = math.sqrt(dx**2 + dy**2)
        
        return angle_diff, distance
    
    def calculate_control_commands(self, current_x: float, current_y: float, 
                                 current_angle: float) -> Tuple[float, float]:
        """
        Calcula comandos de control Ackermann (velocidad + ángulo) para dirigirse al objetivo.
        
        Esta es la interfaz principal que funciona con cualquier tipo de robot.
        
        Args:
            current_x: Posición X actual
            current_y: Posición Y actual  
            current_angle: Ángulo actual del robot
            
        Returns:
            Tuple con (velocidad_lineal, ángulo_de_dirección)
        """
        angle_diff, distance = self.calculate_direction_to_target(
            current_x, current_y, current_angle
        )
        
        if distance < self.tolerance:  # Ya llegamos al objetivo
            return 0.0, 0.0
        
        # Velocidad proporcional a la distancia
        drive_speed = min(self.max_speed, distance * self.linear_gain)
        
        # Ángulo de dirección proporcional al error angular (limitado a ±0.5 rad ≈ ±30°)
        steering_angle = max(-0.5, min(0.5, angle_diff * self.angular_gain))
        
        return drive_speed, steering_angle
    
    def is_target_reached(self, current_x: float, current_y: float) -> bool:
        """
        Verifica si se ha alcanzado el objetivo.
        
        Args:
            current_x: Posición X actual
            current_y: Posición Y actual
            tolerance: Tolerancia de proximidad
            
        Returns:
            True si se alcanzó el objetivo
        """
        if self.target_x is None or self.target_y is None:
            return False
        
        distance = math.sqrt(
            (self.target_x - current_x)**2 + (self.target_y - current_y)**2
        )
        
        return distance < self.tolerance

if __name__ == "__main__":
    # Ejemplo de uso del controlador de navegación
    print("=== NavigationController con comandos Ackermann ===")
    controller = NavigationController(max_speed=1.0, linear_gain=1.5, angular_gain=1.0)
    controller.set_target(2.0, 2.0, tol=0.1)

    # Estado inicial del robot
    current_x, current_y, current_angle = 0.0, 0.0, 0.0

    # Simulación usando comandos Ackermann unificados
    for i in range(15):
        if controller.is_target_reached(current_x, current_y):
            break
            
        drive_speed, steering_angle = controller.calculate_control_commands(
            current_x, current_y, current_angle
        )
        print(f"Paso {i+1}: Pos=({current_x:.2f}, {current_y:.2f}), "
              f"Velocidad={drive_speed:.2f}, Dirección={steering_angle:.2f}")

        # Simulación simple usando modelo Ackermann
        dt = 0.1
        current_x += drive_speed * math.cos(current_angle) * dt
        current_y += drive_speed * math.sin(current_angle) * dt
        current_angle += steering_angle * dt  # Modelo simplificado

    print("¡Navegación completada!")