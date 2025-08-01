"""
Interfaz específica para robots de tracción diferencial.
"""

from abc import abstractmethod
from typing import Tuple
from .irobot_base import IRobotBase


class IDifferentialRobot(IRobotBase):
    """
    Interfaz específica para robots de tracción diferencial.
    
    Los robots diferenciales usan la interfaz Ackermann estándar:
    - set_drive_speed() / get_drive_speed() - Velocidad lineal
    - set_steering_angle() / get_steering_angle() - Ángulo de dirección
    
    La implementación debe convertir internamente estas a velocidades de motores.
    
    Métodos adicionales para acceso directo a motores (opcional):
    """
    
    def __init__(self, wheel_radius: float = 0.0205, wheel_base: float = 0.117):
        """
        Inicializa el robot diferencial con estado de motores y parámetros físicos.
        
        Args:
            wheel_radius: Radio de la rueda (metros)
            wheel_base: Distancia entre ruedas (metros)
        """
        super().__init__()  # Inicializar IRobotBase (pose y estado básico)
        # Estado específico de robots diferenciales
        self.left_speed = 0.0   # Velocidad angular rueda izquierda (rad/s)
        self.right_speed = 0.0  # Velocidad angular rueda derecha (rad/s)
        # Parámetros físicos comunes
        self.wheel_radius = wheel_radius
        self.wheel_base = wheel_base
    
    # Métodos auxiliares para acceso directo a motores
    def set_motor_speeds(self, left_speed: float, right_speed: float) -> None:
        """
        Establece las velocidades de los motores directamente (método auxiliar).
        
        Args:
            left_speed: Velocidad del motor izquierdo (rad/s)
            right_speed: Velocidad del motor derecho (rad/s)
        """
        self.left_speed = left_speed
        self.right_speed = right_speed

    def get_motor_speeds(self) -> Tuple[float, float]:
        """
        Obtiene las velocidades actuales de los motores (método auxiliar).
        
        Returns:
            Tupla (velocidad_izquierda, velocidad_derecha) en rad/s
        """
        return (self.left_speed, self.right_speed)
    
    # Métodos de la interfaz Ackermann (sobrescriben IRobotBase con conversiones)
    def set_drive_speed(self, speed: float) -> None:
        """
        Establece la velocidad de avance (interfaz Ackermann principal).
        Convierte a velocidades diferenciales manteniendo dirección actual.
        
        Args:
            speed: Velocidad lineal en m/s
        """
        # Convertir velocidad lineal a velocidad angular de ruedas
        angular_speed = speed / self.wheel_radius
        
        # Mantener la diferencia de velocidades actual (dirección)
        speed_diff = self.right_speed - self.left_speed
        
        # Establecer velocidades simétricas alrededor de la velocidad objetivo
        self.left_speed = angular_speed - speed_diff / 2.0
        self.right_speed = angular_speed + speed_diff / 2.0
        
        # Actualizar estado base
        self.drive_speed = speed
    
    def set_steering_angle(self, angle: float) -> None:
        """
        Establece el ángulo de dirección (interfaz Ackermann principal).
        Convierte a diferencia de velocidades entre ruedas.
        
        Args:
            angle: Ángulo de dirección en radianes
        """
        # Convertir ángulo a diferencia de velocidades
        # Usar velocidad promedio actual
        avg_angular_speed = (self.left_speed + self.right_speed) / 2.0
        speed_diff = angle * abs(avg_angular_speed) * 2.0
        
        # Aplicar diferencia de velocidades
        self.left_speed = avg_angular_speed - speed_diff / 2.0
        self.right_speed = avg_angular_speed + speed_diff / 2.0
        
        # Actualizar estado base
        self.steering_angle = angle
    
    def get_drive_speed(self) -> float:
        """
        Obtiene la velocidad de avance actual (interfaz Ackermann).
        
        Returns:
            Velocidad lineal promedio en m/s
        """
        avg_angular_speed = (self.left_speed + self.right_speed) / 2.0
        return avg_angular_speed * self.wheel_radius
    
    def get_steering_angle(self) -> float:
        """
        Obtiene el ángulo de dirección actual (interfaz Ackermann).
        
        Returns:
            Ángulo de dirección estimado en radianes
        """
        # Estimar ángulo basado en diferencia de velocidades
        speed_diff = self.right_speed - self.left_speed
        avg_speed = (self.left_speed + self.right_speed) / 2.0
        
        if abs(avg_speed) > 0.001:  # Evitar división por cero
            return speed_diff / (abs(avg_speed) * 2.0)
        return speed_diff / 2.0 if abs(speed_diff) > 0.001 else 0.0
    
    def stop(self) -> None:
        """Detiene el robot (sobrescribe para incluir motores específicos)."""
        super().stop()  # Llama a IRobotBase.stop()
        self.left_speed = 0.0
        self.right_speed = 0.0
