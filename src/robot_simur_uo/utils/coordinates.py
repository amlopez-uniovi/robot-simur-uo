"""
Sistema de coordenadas y transformaciones para robots.
"""

import math
from typing import Tuple, List, Optional
from .math_utils import MathUtils


class CoordinateSystem:
    """Manejo de sistemas de coordenadas y transformaciones."""
    
    def __init__(self, origin: Tuple[float, float] = (0.0, 0.0), 
                 orientation: float = 0.0):
        """
        Inicializa un sistema de coordenadas.
        
        Args:
            origin: Origen del sistema (x, y)
            orientation: Orientación del sistema en radianes
        """
        self.origin = origin
        self.orientation = orientation
    
    def transform_to_global(self, local_point: Tuple[float, float]) -> Tuple[float, float]:
        """
        Transforma un punto del sistema local al global.
        
        Args:
            local_point: Punto en coordenadas locales (x, y)
            
        Returns:
            Punto en coordenadas globales (x, y)
        """
        local_x, local_y = local_point
        
        # Rotar
        cos_theta = math.cos(self.orientation)
        sin_theta = math.sin(self.orientation)
        
        rotated_x = local_x * cos_theta - local_y * sin_theta
        rotated_y = local_x * sin_theta + local_y * cos_theta
        
        # Trasladar
        global_x = rotated_x + self.origin[0]
        global_y = rotated_y + self.origin[1]
        
        return (global_x, global_y)
    
    def transform_to_local(self, global_point: Tuple[float, float]) -> Tuple[float, float]:
        """
        Transforma un punto del sistema global al local.
        
        Args:
            global_point: Punto en coordenadas globales (x, y)
            
        Returns:
            Punto en coordenadas locales (x, y)
        """
        global_x, global_y = global_point
        
        # Trasladar
        translated_x = global_x - self.origin[0]
        translated_y = global_y - self.origin[1]
        
        # Rotar (rotación inversa)
        cos_theta = math.cos(-self.orientation)
        sin_theta = math.sin(-self.orientation)
        
        local_x = translated_x * cos_theta - translated_y * sin_theta
        local_y = translated_x * sin_theta + translated_y * cos_theta
        
        return (local_x, local_y)
    
    def update_pose(self, new_origin: Tuple[float, float], new_orientation: float):
        """
        Actualiza la pose del sistema de coordenadas.
        
        Args:
            new_origin: Nueva posición del origen
            new_orientation: Nueva orientación
        """
        self.origin = new_origin
        self.orientation = MathUtils.normalize_angle(new_orientation)
    
    def get_transformation_matrix(self) -> List[List[float]]:
        """
        Obtiene la matriz de transformación homogénea 3x3.
        
        Returns:
            Matriz de transformación como lista de listas
        """
        cos_theta = math.cos(self.orientation)
        sin_theta = math.sin(self.orientation)
        tx, ty = self.origin
        
        return [
            [cos_theta, -sin_theta, tx],
            [sin_theta,  cos_theta, ty],
            [0,         0,          1]
        ]


class RobotPose:
    """Representa la pose (posición y orientación) de un robot."""
    
    def __init__(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0):
        """
        Inicializa la pose del robot.
        
        Args:
            x: Posición X
            y: Posición Y
            theta: Orientación en radianes
        """
        self.x = x
        self.y = y
        self.theta = MathUtils.normalize_angle(theta)
    
    def update(self, delta_x: float, delta_y: float, delta_theta: float):
        """
        Actualiza la pose con incrementos.
        
        Args:
            delta_x: Incremento en X
            delta_y: Incremento en Y
            delta_theta: Incremento en orientación
        """
        self.x += delta_x
        self.y += delta_y
        self.theta = MathUtils.normalize_angle(self.theta + delta_theta)
    
    def set_pose(self, x: float, y: float, theta: float):
        """
        Establece una nueva pose.
        
        Args:
            x: Nueva posición X
            y: Nueva posición Y
            theta: Nueva orientación
        """
        self.x = x
        self.y = y
        self.theta = MathUtils.normalize_angle(theta)
    
    def distance_to(self, other_pose: 'RobotPose') -> float:
        """
        Calcula la distancia a otra pose.
        
        Args:
            other_pose: Otra pose del robot
            
        Returns:
            Distancia euclidiana
        """
        return MathUtils.distance_2d((self.x, self.y), (other_pose.x, other_pose.y))
    
    def angle_to(self, target_point: Tuple[float, float]) -> float:
        """
        Calcula el ángulo hacia un punto objetivo.
        
        Args:
            target_point: Punto objetivo (x, y)
            
        Returns:
            Ángulo hacia el objetivo en radianes
        """
        dx = target_point[0] - self.x
        dy = target_point[1] - self.y
        return math.atan2(dy, dx)
    
    def angular_difference_to(self, target_point: Tuple[float, float]) -> float:
        """
        Calcula la diferencia angular hacia un punto objetivo.
        
        Args:
            target_point: Punto objetivo (x, y)
            
        Returns:
            Diferencia angular en radianes
        """
        target_angle = self.angle_to(target_point)
        return MathUtils.angle_difference(self.theta, target_angle)
    
    def get_front_point(self, distance: float) -> Tuple[float, float]:
        """
        Obtiene un punto al frente del robot a una distancia dada.
        
        Args:
            distance: Distancia al frente
            
        Returns:
            Punto frontal (x, y)
        """
        front_x = self.x + distance * math.cos(self.theta)
        front_y = self.y + distance * math.sin(self.theta)
        return (front_x, front_y)
    
    def copy(self) -> 'RobotPose':
        """
        Crea una copia de la pose.
        
        Returns:
            Nueva instancia de RobotPose
        """
        return RobotPose(self.x, self.y, self.theta)
    
    def to_tuple(self) -> Tuple[float, float, float]:
        """
        Convierte la pose a tupla.
        
        Returns:
            Tupla (x, y, theta)
        """
        return (self.x, self.y, self.theta)
    
    def __str__(self) -> str:
        """Representación string de la pose."""
        return f"RobotPose(x={self.x:.3f}, y={self.y:.3f}, theta={self.theta:.3f})"


class TrajectoryPoint:
    """Punto en una trayectoria con pose y velocidad."""
    
    def __init__(self, pose: RobotPose, linear_velocity: float = 0.0, 
                 angular_velocity: float = 0.0, timestamp: float = 0.0):
        """
        Inicializa un punto de trayectoria.
        
        Args:
            pose: Pose del robot
            linear_velocity: Velocidad lineal
            angular_velocity: Velocidad angular
            timestamp: Marca temporal
        """
        self.pose = pose
        self.linear_velocity = linear_velocity
        self.angular_velocity = angular_velocity
        self.timestamp = timestamp


class Trajectory:
    """Representación de una trayectoria de robot."""
    
    def __init__(self):
        """Inicializa una trayectoria vacía."""
        self.points: List[TrajectoryPoint] = []
    
    def add_point(self, pose: RobotPose, linear_vel: float = 0.0, 
                  angular_vel: float = 0.0, timestamp: float = 0.0):
        """
        Añade un punto a la trayectoria.
        
        Args:
            pose: Pose del robot
            linear_vel: Velocidad lineal
            angular_vel: Velocidad angular
            timestamp: Marca temporal
        """
        point = TrajectoryPoint(pose, linear_vel, angular_vel, timestamp)
        self.points.append(point)
    
    def get_length(self) -> float:
        """
        Calcula la longitud total de la trayectoria.
        
        Returns:
            Longitud en metros
        """
        if len(self.points) < 2:
            return 0.0
        
        total_length = 0.0
        for i in range(1, len(self.points)):
            distance = self.points[i-1].pose.distance_to(self.points[i].pose)
            total_length += distance
        
        return total_length
    
    def get_duration(self) -> float:
        """
        Calcula la duración total de la trayectoria.
        
        Returns:
            Duración en segundos
        """
        if len(self.points) < 2:
            return 0.0
        
        return self.points[-1].timestamp - self.points[0].timestamp
    
    def interpolate_at_time(self, target_time: float) -> Optional[TrajectoryPoint]:
        """
        Interpola la trayectoria en un tiempo específico.
        
        Args:
            target_time: Tiempo objetivo
            
        Returns:
            Punto interpolado o None si está fuera del rango
        """
        if not self.points:
            return None
        
        # Encontrar puntos adyacentes
        for i in range(len(self.points) - 1):
            if (self.points[i].timestamp <= target_time <= 
                self.points[i+1].timestamp):
                
                # Interpolar entre puntos i e i+1
                t1 = self.points[i].timestamp
                t2 = self.points[i+1].timestamp
                
                if abs(t2 - t1) < 1e-6:  # Evitar división por cero
                    return self.points[i]
                
                # Factor de interpolación
                alpha = (target_time - t1) / (t2 - t1)
                
                # Interpolar pose
                x = MathUtils.lerp(self.points[i].pose.x, self.points[i+1].pose.x, alpha)
                y = MathUtils.lerp(self.points[i].pose.y, self.points[i+1].pose.y, alpha)
                
                # Interpolar ángulo (cuidado con la circularidad)
                theta1 = self.points[i].pose.theta
                theta2 = self.points[i+1].pose.theta
                theta_diff = MathUtils.angle_difference(theta1, theta2)
                theta = MathUtils.normalize_angle(theta1 + alpha * theta_diff)
                
                # Interpolar velocidades
                linear_vel = MathUtils.lerp(
                    self.points[i].linear_velocity, 
                    self.points[i+1].linear_velocity, 
                    alpha
                )
                angular_vel = MathUtils.lerp(
                    self.points[i].angular_velocity, 
                    self.points[i+1].angular_velocity, 
                    alpha
                )
                
                interpolated_pose = RobotPose(x, y, theta)
                return TrajectoryPoint(interpolated_pose, linear_vel, angular_vel, target_time)
        
        return None
    
    def clear(self):
        """Limpia todos los puntos de la trayectoria."""
        self.points.clear()
