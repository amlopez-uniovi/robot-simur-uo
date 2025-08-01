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
    def set_drive_speed(self, speed: float) -> None:
        """
        Establece la velocidad de avance del robot.
        
        Args:
            speed: Velocidad lineal (m/s)
        """
        pass
    
    @abstractmethod
    def set_steering_angle(self, angle: float) -> None:
        """
        Establece el ángulo de dirección del robot.
        
        Args:
            angle: Ángulo de dirección en radianes
        """
        pass
    
    @abstractmethod
    def get_drive_speed(self) -> float:
        """
        Obtiene la velocidad de avance actual.
        
        Returns:
            Velocidad lineal actual (m/s)
        """
        pass
    
    @abstractmethod
    def get_steering_angle(self) -> float:
        """
        Obtiene el ángulo de dirección actual.
        
        Returns:
            Ángulo de dirección actual en radianes
        """
        pass
    
    @abstractmethod
    def cleanup(self) -> None:
        """Limpieza del robot."""
        pass
