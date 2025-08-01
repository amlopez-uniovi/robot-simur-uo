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
        # Estado básico del robot (será sobrescrito por subclases especializadas)
        # Los robots diferenciales usarán left_speed/right_speed
        # Los robots Ackermann usarán forward_speed/steering_angle
        self.forward_speed = 0.0      # Velocidad de avance (m/s) - solo para robots no diferenciales
        self.steering_speed = 0.0   # Velocidad de dirección (rad/s) - solo para robots no diferenciales
    
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
    
    def set_drive_command(self, forward_speed: float, steering_speed: float) -> None:
        """
        Establece la velocidad de avance y dirección simultáneamente (interfaz principal).
        
        Args:
            forward_speed: Velocidad lineal (m/s)
            steering_speed: Velocidad de dirección (rad/s)
        """
        self.forward_speed = forward_speed
        self.steering_speed = steering_speed
    
    def set_forward_speed(self, speed: float) -> None:
        """
        Establece la velocidad de avance del robot manteniendo la velocidad de dirección.
        
        Args:
            speed: Velocidad lineal (m/s)
        """
        self.set_drive_command(speed, self.steering_speed)
    
    def set_steering_speed(self, speed: float) -> None:
        """
        Establece la velocidad de dirección del robot manteniendo la velocidad de avance.
        
        Args:
            speed: Velocidad de dirección en rad/s
        """
        self.set_drive_command(self.forward_speed, speed)
    
    def get_forward_speed(self) -> float:
        """
        Obtiene la velocidad de avance actual.
        
        Returns:
            Velocidad lineal actual (m/s)
        """
        return self.forward_speed
    
    def get_steering_speed(self) -> float:
        """
        Obtiene la velocidad de dirección actual.
        
        Returns:
            Velocidad de dirección actual en rad/s
        """
        return self.steering_speed
    
    def stop(self) -> None:
        """Detiene el robot."""
        self.forward_speed = 0.0
        self.steering_speed = 0.0
    
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
