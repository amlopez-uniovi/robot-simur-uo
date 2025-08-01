"""
Interfaces comunes para todos los tipos de robots.
"""

from abc import ABC, abstractmethod
from typing import Tuple
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


# Mantenemos IRobot como alias por compatibilidad hacia atrás
IRobot = IDifferentialRobot
