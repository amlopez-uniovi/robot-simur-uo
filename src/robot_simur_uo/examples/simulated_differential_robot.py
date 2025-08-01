"""
Ejemplo de implementación simulada de robot diferencial.
Implementación completa con toda la funcionalidad.
"""

import math
from typing import Tuple
from ..utils.coordinates import RobotPose
from ..interfaces.idifferential_robot import IDifferentialRobot


class SimulatedDifferentialRobot(IDifferentialRobot):
    """
    Implementación simulada completa de robot diferencial.
    
    Incluye toda la funcionalidad común más la simulación cinemática.
    """
    
    def __init__(self, wheel_radius: float = 0.025, wheel_base: float = 0.053):
        """
        Inicializa el robot diferencial simulado.
        
        Args:
            wheel_radius: Radio de las ruedas en metros
            wheel_base: Distancia entre ruedas en metros
        """
        self.wheel_radius = wheel_radius
        self.wheel_base = wheel_base
        
        # Estado del robot
        self.pose = RobotPose(0.0, 0.0, 0.0)
        self.left_speed = 0.0   # Velocidad angular rueda izquierda (rad/s)
        self.right_speed = 0.0  # Velocidad angular rueda derecha (rad/s)
    
    def set_motor_speeds(self, left_speed: float, right_speed: float) -> None:
        """Establece las velocidades de los motores."""
        self.left_speed = left_speed
        self.right_speed = right_speed
    
    def get_motor_speeds(self) -> Tuple[float, float]:
        """Obtiene las velocidades actuales de los motores."""
        return (self.left_speed, self.right_speed)
    
    def get_pose(self) -> RobotPose:
        """Obtiene la pose actual del robot."""
        return self.pose.copy()
    
    def set_pose(self, x: float, y: float, theta: float) -> None:
        """Establece la pose del robot."""
        self.pose = RobotPose(x, y, theta)
    
    def stop(self) -> None:
        """Detiene el robot."""
        self.left_speed = 0.0
        self.right_speed = 0.0
    
    def move_forward(self, speed: float = 1.0) -> None:
        """
        Mueve el robot hacia adelante.
        
        Args:
            speed: Velocidad angular de las ruedas en rad/s
        """
        abs_speed = abs(speed)
        self.left_speed = abs_speed
        self.right_speed = abs_speed
    
    def move_backward(self, speed: float = 1.0) -> None:
        """
        Mueve el robot hacia atrás.
        
        Args:
            speed: Velocidad angular de las ruedas en rad/s
        """
        abs_speed = abs(speed)
        self.left_speed = -abs_speed
        self.right_speed = -abs_speed
    
    def turn_left(self, speed: float = 1.0) -> None:
        """
        Gira el robot a la izquierda.
        
        Args:
            speed: Velocidad angular de las ruedas en rad/s
        """
        abs_speed = abs(speed)
        self.left_speed = -abs_speed
        self.right_speed = abs_speed
    
    def turn_right(self, speed: float = 1.0) -> None:
        """
        Gira el robot a la derecha.
        
        Args:
            speed: Velocidad angular de las ruedas en rad/s
        """
        abs_speed = abs(speed)
        self.left_speed = abs_speed
        self.right_speed = -abs_speed
    
    def cleanup(self) -> None:
        """Limpieza del robot."""
        self.stop()
    
    def step(self, dt: float) -> None:
        """
        Implementación simulada del paso de tiempo.
        
        Usa el modelo cinemático diferencial para actualizar la pose.
        
        Args:
            dt: Paso de tiempo en segundos
        """
        # Convertir velocidades angulares a velocidades lineales
        v_left = self.left_speed * self.wheel_radius
        v_right = self.right_speed * self.wheel_radius
        
        # Calcular velocidades del robot
        v_linear = (v_left + v_right) / 2
        v_angular = (v_right - v_left) / self.wheel_base
        
        # Obtener orientación actual
        current_theta = self.pose.theta
        
        # Calcular cambios de posición
        dx = v_linear * math.cos(current_theta) * dt
        dy = v_linear * math.sin(current_theta) * dt
        dtheta = v_angular * dt
        
        # Actualizar pose
        self.pose.update(dx, dy, dtheta)
    
    def __str__(self) -> str:
        """Representación en string del robot."""
        return f"SimulatedDifferentialRobot(pose={self.pose}, left={self.left_speed:.2f}, right={self.right_speed:.2f})"
