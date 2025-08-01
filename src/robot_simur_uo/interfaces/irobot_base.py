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
    
    Implementa directamente la gestión de pose que es común para todos los robots.
    """
    
    def __init__(self):
        """Inicializa el robot con pose y estado básico."""
        self.pose = RobotPose(0.0, 0.0, 0.0)
        # Estado básico del robot (común para todos los tipos)
        self.drive_speed = 0.0      # Velocidad de avance (m/s)
        self.steering_angle = 0.0   # Ángulo de dirección (radianes)
    
    # Métodos implementados (comunes para todos los robots)
    def get_pose(self) -> RobotPose:
        """
        Obtiene la pose actual del robot.
        
        Returns:
            Pose actual del robot
        """
        return self.pose.copy()
    
    def set_pose(self, x: float, y: float, theta: float) -> None:
        """
        Establece la pose del robot.
        
        Args:
            x: Coordenada x
            y: Coordenada y  
            theta: Ángulo de orientación
        """
        self.pose = RobotPose(x, y, theta)
    
    def set_drive_speed(self, speed: float) -> None:
        """
        Establece la velocidad de avance del robot.
        
        Args:
            speed: Velocidad lineal (m/s)
        """
        self.drive_speed = speed
    
    def set_steering_angle(self, angle: float) -> None:
        """
        Establece el ángulo de dirección del robot.
        
        Args:
            angle: Ángulo de dirección en radianes
        """
        self.steering_angle = angle
    
    def get_drive_speed(self) -> float:
        """
        Obtiene la velocidad de avance actual.
        
        Returns:
            Velocidad lineal actual (m/s)
        """
        return self.drive_speed
    
    def get_steering_angle(self) -> float:
        """
        Obtiene el ángulo de dirección actual.
        
        Returns:
            Ángulo de dirección actual en radianes
        """
        return self.steering_angle
    
    def stop(self) -> None:
        """Detiene el robot."""
        self.drive_speed = 0.0
        self.steering_angle = 0.0
    
    def cleanup(self) -> None:
        """Limpieza del robot."""
        self.stop()
    
    # Métodos abstractos (deben ser implementados por las subclases)
    @abstractmethod
    def step(self, dt: float) -> None:
        """
        Ejecuta un paso de simulación.
        
        Args:
            dt: Paso de tiempo en segundos
        """
        pass
