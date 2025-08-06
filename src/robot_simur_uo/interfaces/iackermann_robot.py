"""
Interfaz específica para robots con dirección tipo Ackermann.

Maneja la conversión entre steering_speed (usado por controladores) 
y steering_angle (usado por la cinemática Ackermann).
"""

from abc import abstractmethod
from .irobot_base import IRobotBase


class IAckermannRobot(IRobotBase):
    """
    Interfaz específica para robots con dirección tipo Ackermann.
    
    Los robots Ackermann tienen una diferencia conceptual importante:
    - NavigationController devuelve steering_speed (rad/s)
    - Robots Ackermann usan steering_angle (rad) para las ruedas directrices
    
    Esta clase maneja la conversión entre steering_speed y steering_angle,
    y proporciona métodos específicos para robots Ackermann.
    """
    
    def __init__(self, max_steering_angle: float = 0.5):
        """Inicializa el robot Ackermann con ángulo de dirección."""
        super().__init__()
        self.steering_angle = 0.0  # Ángulo actual de las ruedas directrices (rad)
        self.max_steering_angle = max_steering_angle  # Ángulo máximo (rad)

    # Métodos auxiliares para acceso directo a motores
    @abstractmethod
    def set_motor_velocities(self, forward_speed: float, steering_angle: float) -> None:
        """
        Método abstracto: debe ser implementado por cada robot concreto para aplicar físicamente las velocidades a los motores.
        Args:
            forward_speed: Velocidad de avance (m/s)
            steering_angle: Ángulo de dirección (rad)
        """
        pass

    
    def set_drive_command(self, forward_speed: float, steering_speed: float) -> None:
        """
        Interfaz unificada que convierte steering_speed a steering_angle internamente.
        
        Args:
            forward_speed: Velocidad lineal (m/s)
            steering_speed: Velocidad de dirección (rad/s) - se convierte a ángulo
        """
        # Conversión de steering_speed a steering_angle
        # Simplificación: asumir dt=1s para la conversión básica
        # En implementaciones reales, esto podría ser más sofisticado
        desired_angle = max(-self.max_steering_angle, 
                                 min(self.max_steering_angle, steering_speed))  # Conversión simplificada
        self.set_motor_velocities(forward_speed, desired_angle)
    

