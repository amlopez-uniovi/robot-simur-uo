"""
Ejemplo de implementación simulada de robot Ackermann.
Implementación completa con toda la funcionalidad.
"""

import math
from ..utils.coordinates import RobotPose
from ..interfaces.iackermann_robot import IAckermannRobot


class SimulatedAckermannRobot(IAckermannRobot):
    """
    Implementación simulada completa de robot tipo Ackermann.
    
    Incluye toda la funcionalidad común más la simulación cinemática.
    """
    
    def __init__(self, wheelbase: float = 0.25, max_steering_angle: float = math.pi/6):
        """
        Inicializa el robot Ackermann simulado.
        
        Args:
            wheelbase: Distancia entre ejes (metros)
            max_steering_angle: Ángulo máximo de dirección (radianes)
        """
        self.wheelbase = wheelbase
        self.max_steering_angle = max_steering_angle
        
        # Estado del robot
        self.pose = RobotPose(0.0, 0.0, 0.0)
        self.steering_angle = 0.0  # Ángulo de dirección actual
        self.drive_speed = 0.0     # Velocidad de tracción actual
        
    def set_steering_angle(self, angle: float) -> None:
        """Establece el ángulo de dirección."""
        # Limitar el ángulo dentro del rango permitido
        self.steering_angle = max(-self.max_steering_angle, 
                                min(self.max_steering_angle, angle))
    
    def set_drive_speed(self, speed: float) -> None:
        """Establece la velocidad de tracción."""
        self.drive_speed = speed
    
    def get_steering_angle(self) -> float:
        """Obtiene el ángulo de dirección actual."""
        return self.steering_angle
    
    def get_drive_speed(self) -> float:
        """Obtiene la velocidad de tracción actual."""
        return self.drive_speed
    
    def get_pose(self) -> RobotPose:
        """Obtiene la pose actual del robot."""
        return self.pose.copy()
    
    def set_pose(self, x: float, y: float, theta: float) -> None:
        """Establece la pose del robot."""
        self.pose = RobotPose(x, y, theta)
    
    def stop(self) -> None:
        """Detiene el robot."""
        self.drive_speed = 0.0
        self.steering_angle = 0.0
    
    def cleanup(self) -> None:
        """Limpieza del robot."""
        self.stop()
    
    def step(self, dt: float) -> None:
        """
        Implementación simulada del paso de tiempo.
        
        Usa el modelo cinemático de Ackermann para actualizar la pose.
        
        Args:
            dt: Paso de tiempo en segundos
        """
        if abs(self.drive_speed) < 1e-6:
            return  # No hay movimiento
        
        # Obtener pose actual
        x, y, theta = self.pose.to_tuple()
        
        # Modelo cinemático de Ackermann
        if abs(self.steering_angle) < 1e-6:
            # Movimiento recto
            dx = self.drive_speed * math.cos(theta) * dt
            dy = self.drive_speed * math.sin(theta) * dt
            dtheta = 0.0
        else:
            # Movimiento con giro
            # Radio de giro
            R = self.wheelbase / math.tan(self.steering_angle)
            
            # Velocidad angular
            omega = self.drive_speed / R
            
            # Calcular nuevo estado
            dtheta = omega * dt
            dx = R * (math.sin(theta + dtheta) - math.sin(theta))
            dy = R * (-math.cos(theta + dtheta) + math.cos(theta))
        
        # Actualizar pose
        new_x = x + dx
        new_y = y + dy
        new_theta = (theta + dtheta) % (2 * math.pi)
        
        self.pose.x = new_x
        self.pose.y = new_y
        self.pose.theta = new_theta
    
    def __str__(self) -> str:
        """Representación en string del robot."""
        return f"SimulatedAckermannRobot(pose={self.pose}, steering={self.steering_angle:.3f}, speed={self.drive_speed:.3f})"
