"""
Utilidades matemáticas para robots.
"""

import math
from typing import Tuple, List


class MathUtils:
    """Clase con funciones matemáticas útiles para robótica."""
    
    @staticmethod
    def normalize_angle(angle: float) -> float:
        """
        Normaliza un ángulo al rango [-π, π].
        
        Args:
            angle: Ángulo en radianes
            
        Returns:
            Ángulo normalizado
        """
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    @staticmethod
    def angle_difference(angle1: float, angle2: float) -> float:
        """
        Calcula la diferencia más corta entre dos ángulos.
        
        Args:
            angle1: Primer ángulo en radianes
            angle2: Segundo ángulo en radianes
            
        Returns:
            Diferencia angular más corta
        """
        diff = angle2 - angle1
        return MathUtils.normalize_angle(diff)
    
    @staticmethod
    def distance_2d(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
        """
        Calcula la distancia euclidiana entre dos puntos 2D.
        
        Args:
            p1: Primer punto (x, y)
            p2: Segundo punto (x, y)
            
        Returns:
            Distancia entre los puntos
        """
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        return math.sqrt(dx**2 + dy**2)
    
    @staticmethod
    def point_in_circle(point: Tuple[float, float], 
                       center: Tuple[float, float], 
                       radius: float) -> bool:
        """
        Verifica si un punto está dentro de un círculo.
        
        Args:
            point: Punto a verificar (x, y)
            center: Centro del círculo (x, y)
            radius: Radio del círculo
            
        Returns:
            True si el punto está dentro del círculo
        """
        distance = MathUtils.distance_2d(point, center)
        return distance <= radius
    
    @staticmethod
    def clamp(value: float, min_val: float, max_val: float) -> float:
        """
        Limita un valor a un rango específico.
        
        Args:
            value: Valor a limitar
            min_val: Valor mínimo
            max_val: Valor máximo
            
        Returns:
            Valor limitado al rango
        """
        return max(min_val, min(max_val, value))
    
    @staticmethod
    def lerp(a: float, b: float, t: float) -> float:
        """
        Interpolación lineal entre dos valores.
        
        Args:
            a: Valor inicial
            b: Valor final
            t: Factor de interpolación (0-1)
            
        Returns:
            Valor interpolado
        """
        return a + t * (b - a)
    
    @staticmethod
    def moving_average(values: List[float], window_size: int = 3) -> List[float]:
        """
        Calcula media móvil de una serie de valores.
        
        Args:
            values: Lista de valores
            window_size: Tamaño de la ventana
            
        Returns:
            Lista de valores con media móvil aplicada
        """
        if window_size <= 1 or len(values) < window_size:
            return values.copy()
        
        smoothed = []
        for i in range(len(values)):
            start_idx = max(0, i - window_size//2)
            end_idx = min(len(values), i + window_size//2 + 1)
            window_values = values[start_idx:end_idx]
            avg = sum(window_values) / len(window_values)
            smoothed.append(avg)
        
        return smoothed
    
    @staticmethod
    def wrap_to_pi(angle: float) -> float:
        """
        Envuelve un ángulo al rango [-π, π]. Alias para normalize_angle.
        
        Args:
            angle: Ángulo en radianes
            
        Returns:
            Ángulo envuelto
        """
        return MathUtils.normalize_angle(angle)
    
    @staticmethod
    def degrees_to_radians(degrees: float) -> float:
        """
        Convierte grados a radianes.
        
        Args:
            degrees: Ángulo en grados
            
        Returns:
            Ángulo en radianes
        """
        return degrees * math.pi / 180.0
    
    @staticmethod
    def radians_to_degrees(radians: float) -> float:
        """
        Convierte radianes a grados.
        
        Args:
            radians: Ángulo en radianes
            
        Returns:
            Ángulo en grados
        """
        return radians * 180.0 / math.pi
    
    @staticmethod
    def rotate_point(point: Tuple[float, float], 
                    angle: float, 
                    center: Tuple[float, float] = (0, 0)) -> Tuple[float, float]:
        """
        Rota un punto alrededor de un centro.
        
        Args:
            point: Punto a rotar (x, y)
            angle: Ángulo de rotación en radianes
            center: Centro de rotación (x, y)
            
        Returns:
            Punto rotado (x, y)
        """
        # Trasladar al origen
        x = point[0] - center[0]
        y = point[1] - center[1]
        
        # Rotar
        cos_angle = math.cos(angle)
        sin_angle = math.sin(angle)
        
        rotated_x = x * cos_angle - y * sin_angle
        rotated_y = x * sin_angle + y * cos_angle
        
        # Trasladar de vuelta
        return (rotated_x + center[0], rotated_y + center[1])
    
    @staticmethod
    def line_intersection(line1: Tuple[Tuple[float, float], Tuple[float, float]],
                         line2: Tuple[Tuple[float, float], Tuple[float, float]]) -> Tuple[float, float]:
        """
        Encuentra la intersección entre dos líneas.
        
        Args:
            line1: Primera línea ((x1, y1), (x2, y2))
            line2: Segunda línea ((x3, y3), (x4, y4))
            
        Returns:
            Punto de intersección (x, y) o None si son paralelas
        """
        (x1, y1), (x2, y2) = line1
        (x3, y3), (x4, y4) = line2
        
        denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
        
        if abs(denom) < 1e-10:  # Líneas paralelas
            return None
        
        t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom
        
        x = x1 + t * (x2 - x1)
        y = y1 + t * (y2 - y1)
        
        return (x, y)
    
    @staticmethod
    def gaussian_noise(mean: float = 0.0, std_dev: float = 1.0) -> float:
        """
        Genera ruido gaussiano usando el método Box-Muller.
        
        Args:
            mean: Media de la distribución
            std_dev: Desviación estándar
            
        Returns:
            Valor con ruido gaussiano
        """
        import random
        
        # Box-Muller transform
        u1 = random.random()
        u2 = random.random()
        
        z0 = math.sqrt(-2 * math.log(u1)) * math.cos(2 * math.pi * u2)
        
        return mean + std_dev * z0
