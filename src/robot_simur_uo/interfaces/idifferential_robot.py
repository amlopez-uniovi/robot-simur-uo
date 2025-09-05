"""
Interfaz específica para robots de tracción diferencial.

Define la interfaz y métodos comunes para robots diferenciales.
"""

from abc import abstractmethod
from typing import Tuple
from .irobot_base import IRobotBase


class IDifferentialRobot(IRobotBase):
    """
    Interfaz específica para robots de tracción diferencial.

    Los robots diferenciales usan la interfaz estándar:
        - set_drive_command(drive_speed, steering_speed) - Comando unificado principal
        - set_forward_speed() / get_forward_speed() - Velocidad lineal (mantiene velocidad de dirección)
        - set_steering_speed() / get_steering_speed() - Velocidad de dirección (mantiene velocidad lineal)

    La implementación convierte internamente estas a velocidades de motores.

    Métodos adicionales para acceso directo a motores (opcional).
    """
    
    def __init__(self, wheel_radius: float = 0.0205, wheel_base: float = 0.117):
        """
        Inicializa el robot diferencial con estado de motores y parámetros físicos.
        
        Args:
            wheel_radius: Radio de la rueda (metros)
            wheel_base: Distancia entre ruedas (metros)
        """
        super().__init__()  # Inicializar IRobotBase (pose y estado básico)
        # Parámetros físicos comunes
        self.wheel_radius = wheel_radius
        self.wheel_base = wheel_base
    
    # Métodos auxiliares para acceso directo a motores
    @abstractmethod
    def set_motor_velocities(self, left_speed: float, right_speed: float) -> None:
        """
        Método abstracto: debe ser implementado por cada robot concreto para aplicar físicamente las velocidades a los motores.
        Args:
            left_speed: Velocidad del motor izquierdo (rad/s)
            right_speed: Velocidad del motor derecho (rad/s)
        """
        pass

    
    # Métodos de la interfaz (sobrescriben IRobotBase con conversiones)
    def set_drive_command(self, forward_speed: float, steering_speed: float) -> None:
        """
        Establece velocidad de avance y dirección simultáneamente (interfaz principal).
        Convierte a velocidades diferenciales optimizadas.
        
        Args:
            forward_speed: Velocidad lineal en m/s
            steering_speed: Velocidad de dirección en rad/s
        """
        # Convertir velocidad lineal a velocidad angular de ruedas
        angular_speed = forward_speed / self.wheel_radius
        # Calcular diferencia de velocidades basada en la velocidad de dirección
        # Para robots diferenciales: omega = (v_derecha - v_izquierda) / wheel_base
        # Por lo tanto: v_diferencial = steering_speed * wheel_base / 2
        differential_velocity = steering_speed * self.wheel_base / (2.0 * self.wheel_radius)
        # Calcular velocidades de cada rueda
        left_speed = angular_speed - differential_velocity
        right_speed = angular_speed + differential_velocity
        self.set_motor_velocities(left_speed, right_speed)
        # NO almacenar forward_speed y steering_speed redundantemente
        # Se calculan cuando se necesiten usando get_forward_speed() y get_steering_speed()


    
    
    def stop(self) -> None:
        """Detiene el robot (sobrescribe para incluir motores específicos)."""
        super().stop()  # Llama a IRobotBase.stop()
        self.set_motor_velocities(0.0, 0.0)
