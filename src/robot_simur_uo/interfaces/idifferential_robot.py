"""
Interfaz específica para robots de tracción diferencial.
"""

from abc import abstractmethod
from typing import Tuple
from .irobot_base import IRobotBase


class IDifferentialRobot(IRobotBase):
    """
    Interfaz específica para robots de tracción diferencial.
    
    Define métodos específicos para robots con dos ruedas independientes.
    """
    
    @abstractmethod
    def set_motor_speeds(self, left_speed: float, right_speed: float) -> None:
        """
        Establece las velocidades de los motores.
        
        Args:
            left_speed: Velocidad del motor izquierdo (rad/s)
            right_speed: Velocidad del motor derecho (rad/s)
        """
        pass
    
    @abstractmethod
    def get_motor_speeds(self) -> Tuple[float, float]:
        """
        Obtiene las velocidades actuales de los motores.
        
        Returns:
            Tupla (velocidad_izquierda, velocidad_derecha) en rad/s
        """
        pass
