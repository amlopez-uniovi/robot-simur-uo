"""
Interfaz específica para robots con dirección tipo Ackermann.

Maneja la conversión entre steering_speed (usado por controladores) 
y steering_angle (usado por la cinemática Ackermann).
"""

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
    
    def __init__(self):
        """Inicializa el robot Ackermann con ángulo de dirección."""
        super().__init__()
        self.steering_angle = 0.0  # Ángulo actual de las ruedas directrices (rad)
        self.max_steering_angle = 0.5  # Ángulo máximo por defecto (rad) ~28.6°
    
    def set_drive_command(self, forward_speed: float, steering_speed: float) -> None:
        """
        Interfaz unificada que convierte steering_speed a steering_angle internamente.
        
        Args:
            forward_speed: Velocidad lineal (m/s)
            steering_speed: Velocidad de dirección (rad/s) - se convierte a ángulo
        """
        super().set_drive_command(forward_speed, steering_speed)
        # Conversión de steering_speed a steering_angle
        # Simplificación: asumir dt=1s para la conversión básica
        # En implementaciones reales, esto podría ser más sofisticado
        desired_angle = steering_speed  # Conversión simplificada
        self.set_steering_angle(desired_angle)
    
    def set_steering_angle(self, angle: float) -> None:
        """
        Establece el ángulo de dirección con limitación (método específico Ackermann).
        
        Args:
            angle: Ángulo de dirección en radianes
        """
        # Limitar el ángulo dentro del rango permitido
        self.steering_angle = max(-self.max_steering_angle, 
                                 min(self.max_steering_angle, angle))
    
    def get_steering_angle(self) -> float:
        """
        Obtiene el ángulo de dirección actual.
        
        Returns:
            Ángulo de dirección actual en radianes
        """
        return self.steering_angle
    
    def set_max_steering_angle(self, max_angle: float) -> None:
        """
        Establece el ángulo máximo de dirección.
        
        Args:
            max_angle: Ángulo máximo en radianes
        """
        self.max_steering_angle = abs(max_angle)
    
    def get_max_steering_angle(self) -> float:
        """
        Obtiene el ángulo máximo de dirección.
        
        Returns:
            Ángulo máximo en radianes
        """
        return self.max_steering_angle
