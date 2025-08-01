"""
Interfaz específica para robots con dirección tipo Ackermann.
"""

from .irobot_base import IRobotBase


class IAckermannRobot(IRobotBase):
    """
    Interfaz específica para robots con dirección tipo Ackermann.
    
    Los robots Ackermann usan directamente los métodos de la interfaz base:
    - set_drive_speed() / get_drive_speed()
    - set_steering_angle() / get_steering_angle()
    
    No requieren métodos adicionales ya que el modelo Ackermann es la base.
    """
    pass

from abc import abstractmethod
from .irobot_base import IRobotBase


class IAckermannRobot(IRobotBase):
    """
    Interfaz específica para robots con dirección tipo Ackermann.
    
    Define métodos específicos para robots como coches con dirección frontal.
    """
    
    @abstractmethod
    def set_steering_angle(self, angle: float) -> None:
        """
        Establece el ángulo de dirección.
        
        Args:
            angle: Ángulo de dirección en radianes
        """
        pass
    
    @abstractmethod
    def set_drive_speed(self, speed: float) -> None:
        """
        Establece la velocidad de tracción.
        
        Args:
            speed: Velocidad lineal en m/s
        """
        pass
    
    @abstractmethod
    def get_steering_angle(self) -> float:
        """
        Obtiene el ángulo de dirección actual.
        
        Returns:
            Ángulo de dirección en radianes
        """
        pass
    
    @abstractmethod
    def get_drive_speed(self) -> float:
        """
        Obtiene la velocidad de tracción actual.
        
        Returns:
            Velocidad lineal en m/s
        """
        pass
