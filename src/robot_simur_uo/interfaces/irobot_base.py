"""
Interfaz base para todos los tipos de robots.
"""

from abc import ABC, abstractmethod
from ..utils.coordinates import RobotPose


class IRobotBase(ABC):
    """
    Interfaz base para todos los robots.
    
    Define los métodos comunes que deben implementar todas las clases de robot
    independientemente de su tipo de locomoción.
    """
    
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
    def stop(self) -> None:
        """Detiene el robot."""
        pass
    
    @abstractmethod
    def move_forward(self, speed: float = 2.0) -> None:
        """
        Mueve el robot hacia adelante.
        
        Args:
            speed: Velocidad (unidades dependen del tipo de robot)
        """
        pass
    
    @abstractmethod
    def move_backward(self, speed: float = 2.0) -> None:
        """
        Mueve el robot hacia atrás.
        
        Args:
            speed: Velocidad (unidades dependen del tipo de robot)
        """
        pass
    
    @abstractmethod
    def turn_left(self, speed: float = 2.0) -> None:
        """
        Gira el robot a la izquierda.
        
        Args:
            speed: Velocidad (unidades dependen del tipo de robot)
        """
        pass
    
    @abstractmethod
    def turn_right(self, speed: float = 2.0) -> None:
        """
        Gira el robot a la derecha.
        
        Args:
            speed: Velocidad (unidades dependen del tipo de robot)
        """
        pass
    
    @abstractmethod
    def cleanup(self) -> None:
        """Limpieza del robot."""
        pass
