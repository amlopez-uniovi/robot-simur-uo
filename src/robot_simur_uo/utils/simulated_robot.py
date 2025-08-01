"""
Robot simulado para ejemplos y pruebas que no requieren Webots.
"""

import math
from typing import Tuple
from .coordinates import RobotPose
from ..interfaces.irobot import IDifferentialRobot


class SimulatedDifferentialRobot(IDifferentialRobot):
    """
    Robot diferencial simulado para ejemplos y testing sin dependencia de Webots.
    
    Simula un robot diferencial con dos ruedas independientes.
    """
    
    def __init__(self, wheel_radius: float = 0.025, wheel_base: float = 0.053):
        """
        Inicializa el robot simulado.
        
        Args:
            wheel_radius: Radio de las ruedas en metros
            wheel_base: Distancia entre ruedas en metros
        """
        self.wheel_radius = wheel_radius
        self.wheel_base = wheel_base
        self.pose = RobotPose(0.0, 0.0, 0.0)
        
        # Velocidades actuales de los motores (rad/s)
        self.left_motor_speed = 0.0
        self.right_motor_speed = 0.0
        
        # Límites de velocidad
        self.max_motor_speed = 10.0  # rad/s
        
    def set_motor_speeds(self, left_speed: float, right_speed: float):
        """
        Establece las velocidades de los motores.
        
        Args:
            left_speed: Velocidad del motor izquierdo (rad/s)
            right_speed: Velocidad del motor derecho (rad/s)
        """
        # Limitar velocidades
        self.left_motor_speed = max(-self.max_motor_speed, 
                                   min(self.max_motor_speed, left_speed))
        self.right_motor_speed = max(-self.max_motor_speed, 
                                    min(self.max_motor_speed, right_speed))
    
    def step(self, dt: float):
        """
        Ejecuta un paso de simulación.
        
        Args:
            dt: Paso de tiempo en segundos
        """
        # Calcular velocidades lineales de las ruedas
        v_left = self.left_motor_speed * self.wheel_radius
        v_right = self.right_motor_speed * self.wheel_radius
        
        # Calcular velocidad lineal y angular del robot
        v_linear = (v_left + v_right) / 2
        v_angular = (v_right - v_left) / self.wheel_base
        
        # Integración simple para actualizar pose
        current_theta = self.pose.theta
        
        dx = v_linear * math.cos(current_theta) * dt
        dy = v_linear * math.sin(current_theta) * dt
        dtheta = v_angular * dt
        
        self.pose.update(dx, dy, dtheta)
    
    def get_pose(self) -> RobotPose:
        """
        Obtiene la pose actual del robot.
        
        Returns:
            Pose actual del robot
        """
        return self.pose.copy()
    
    def set_pose(self, x: float, y: float, theta: float):
        """
        Establece la pose del robot.
        
        Args:
            x: Coordenada x
            y: Coordenada y  
            theta: Ángulo de orientación
        """
        self.pose.x = x
        self.pose.y = y
        self.pose.theta = theta
    
    def get_motor_speeds(self) -> Tuple[float, float]:
        """
        Obtiene las velocidades actuales de los motores.
        
        Returns:
            Tupla (velocidad_izquierda, velocidad_derecha) en rad/s
        """
        return self.left_motor_speed, self.right_motor_speed
    
    def stop(self):
        """Detiene el robot."""
        self.left_motor_speed = 0.0
        self.right_motor_speed = 0.0
    
    def move_forward(self, speed: float = 2.0):
        """
        Mueve el robot hacia adelante.
        
        Args:
            speed: Velocidad en rad/s
        """
        self.set_motor_speeds(speed, speed)
    
    def move_backward(self, speed: float = 2.0):
        """
        Mueve el robot hacia atrás.
        
        Args:
            speed: Velocidad en rad/s
        """
        self.set_motor_speeds(-speed, -speed)
    
    def turn_left(self, speed: float = 2.0):
        """
        Gira el robot a la izquierda.
        
        Args:
            speed: Velocidad en rad/s
        """
        self.set_motor_speeds(-speed, speed)
    
    def turn_right(self, speed: float = 2.0):
        """
        Gira el robot a la derecha.
        
        Args:
            speed: Velocidad en rad/s
        """
        self.set_motor_speeds(speed, -speed)
    
    def __str__(self) -> str:
        """Representación en string del robot."""
        return f"SimulatedDifferentialRobot(pose={self.pose}, motors=({self.left_motor_speed:.2f}, {self.right_motor_speed:.2f}))"
