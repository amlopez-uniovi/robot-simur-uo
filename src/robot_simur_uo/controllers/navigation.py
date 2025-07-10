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
    
    def calculate_motor_speeds(self, current_x: float, current_y: float, 
                             current_angle: float, wheel_base: float = 0.1) -> Tuple[float, float]:
        """
        Calcula las velocidades de los motores para dirigirse al objetivo.
        
        Args:
            current_x: Posición X actual
            current_y: Posición Y actual  
            current_angle: Ángulo actual del robot
            wheel_base: Distancia entre ruedas
            
        Returns:
            Tuple con (velocidad_motor_izquierdo, velocidad_motor_derecho)
        """
        angle_diff, distance = self.calculate_direction_to_target(
            current_x, current_y, current_angle
        )
        
        if distance < self.tolerance:  # Ya llegamos al objetivo
            return 0.0, 0.0
        
        # Velocidad base proporcional a la distancia
        base_speed = min(self.max_speed, distance * self.linear_gain)
        
        # Ajuste angular
        angular_speed = angle_diff * self.angular_gain
        
        # Calcular velocidades de motores
        left_speed = base_speed - angular_speed
        right_speed = base_speed + angular_speed
        
        # Limitar velocidades
        left_speed = max(-self.max_speed, min(self.max_speed, left_speed))
        right_speed = max(-self.max_speed, min(self.max_speed, right_speed))
        
        return left_speed, right_speed
    
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
    controller = NavigationController(max_speed=1.0)
    controller.set_target(5.0, 5.0, tol=0.1)

    # Estado inicial del robot
    current_x, current_y, current_angle = 0.0, 0.0, 0.0

    # Simulación del bucle de control
    while not controller.is_target_reached(current_x, current_y):
        left_speed, right_speed = controller.calculate_motor_speeds(
            current_x, current_y, current_angle
        )
        print(f"Posición: ({current_x:.2f}, {current_y:.2f}), "
                f"Ángulo: {current_angle:.2f}, "
                f"Velocidades: Izq={left_speed:.2f}, Der={right_speed:.2f}")

        # Simulación simple del movimiento (no considera física real)
        # Avance proporcional a la velocidad media
        speed = (left_speed + right_speed) / 2.0
        current_x += speed * math.cos(current_angle) * 0.1  # 0.1 = dt
        current_y += speed * math.sin(current_angle) * 0.1

        # Giro proporcional a la diferencia de velocidades
        current_angle += (right_speed - left_speed) / 0.1 * 0.1  # wheel_base=0.1, dt=0.1

    print("¡Objetivo alcanzado!")