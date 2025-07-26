"""
Interfaz común para todos los tipos de robots.
"""

from abc import ABC, abstractmethod
from typing import Tuple
from ..utils.coordinates import RobotPose


class IRobot(ABC):
    """
    Interfaz común para todos los robots (simulados y de Webots).
    
    Define los métodos que deben implementar todas las clases de robot
    para asegurar compatibilidad entre robots simulados y reales.
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
    def step(self, dt: float) -> None:
        """
        Ejecuta un paso de simulación.
        
        Args:
            dt: Paso de tiempo en segundos
        """
        pass
    
    @abstractmethod
    def get_pose(self) -> RobotPose:
        """
        Obtiene la pose actual del robot.
        
        Returns:
            Pose actual del robot
        """
        pass
    
    @abstractmethod
    def set_pose(self, x: float, y: float, theta: float) -> None:
        """
        Establece la pose del robot.
        
        Args:
            x: Coordenada x
            y: Coordenada y  
            theta: Ángulo de orientación
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
    
    @abstractmethod
    def stop(self) -> None:
        """Detiene el robot."""
        pass
    
    @abstractmethod
    def move_forward(self, speed: float = 2.0) -> None:
        """
        Mueve el robot hacia adelante.
        
        Args:
            speed: Velocidad en rad/s
        """
        pass
    
    @abstractmethod
    def move_backward(self, speed: float = 2.0) -> None:
        """
        Mueve el robot hacia atrás.
        
        Args:
            speed: Velocidad en rad/s
        """
        pass
    
    @abstractmethod
    def turn_left(self, speed: float = 2.0) -> None:
        """
        Gira el robot a la izquierda.
        
        Args:
            speed: Velocidad en rad/s
        """
        pass
    
    @abstractmethod
    def turn_right(self, speed: float = 2.0) -> None:
        """
        Gira el robot a la derecha.
        
        Args:
            speed: Velocidad en rad/s
        """
        pass
