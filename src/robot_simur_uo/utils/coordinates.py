"""
Sistema de coordenadas básico para robots.
"""
import math


class RobotPose:
    """Representa la posición y orientación de un robot en el espacio 2D."""
    
    def __init__(self, x=0.0, y=0.0, theta=0.0):
        """
        Args:
            x (float): Posición en el eje X
            y (float): Posición en el eje Y
            theta (float): Orientación en radianes
        """
        self.x = x
        self.y = y
        self.theta = theta
    
    def distance_to(self, other):
        """Calcula la distancia euclidiana a otra posición."""
        dx = other.x - self.x
        dy = other.y - self.y
        return math.sqrt(dx**2 + dy**2)
    
    def angle_to(self, other):
        """Calcula el ángulo hacia otra posición."""
        dx = other.x - self.x
        dy = other.y - self.y
        return math.atan2(dy, dx)
    
    def copy(self):
        """Crea una copia de la pose."""
        return RobotPose(self.x, self.y, self.theta)
    
    def to_tuple(self):
        """Convierte la pose a una tupla (x, y, theta)."""
        return (self.x, self.y, self.theta)
    
    def update(self, dx, dy, dtheta):
        """Actualiza la pose con cambios incrementales."""
        self.x += dx
        self.y += dy
        self.theta += dtheta
    
    def __str__(self):
        return f"RobotPose(x={self.x:.2f}, y={self.y:.2f}, theta={self.theta:.2f})"
    
    def __repr__(self):
        return self.__str__()