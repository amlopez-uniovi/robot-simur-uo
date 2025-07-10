"""
Procesador de datos LiDAR.
"""

import math
from typing import List, Tuple, Optional
import numpy as np


class LidarProcessor:
    """Procesador para datos de sensores LiDAR."""
    
    def __init__(self, max_range: float = 5.0, angle_resolution: float = 0.01):
        """
        Inicializa el procesador LiDAR.
        
        Args:
            max_range: Rango máximo del sensor
            angle_resolution: Resolución angular en radianes
        """
        self.max_range = max_range
        self.angle_resolution = angle_resolution
        
    def filter_noise(self, ranges: List[float], 
                    noise_threshold: float = 0.1) -> List[float]:
        """
        Filtra ruido de las mediciones LiDAR.
        
        Args:
            ranges: Lista de distancias medidas
            noise_threshold: Umbral para filtro de ruido
            
        Returns:
            Lista de distancias filtradas
        """
        if len(ranges) < 3:
            return ranges
        
        filtered = []
        for i in range(len(ranges)):
            current = ranges[i]
            
            # Valores inválidos o fuera de rango
            if current <= 0 or current > self.max_range:
                # Usar promedio de vecinos válidos
                neighbors = []
                if i > 0 and 0 < ranges[i-1] <= self.max_range:
                    neighbors.append(ranges[i-1])
                if i < len(ranges)-1 and 0 < ranges[i+1] <= self.max_range:
                    neighbors.append(ranges[i+1])
                
                if neighbors:
                    filtered.append(sum(neighbors) / len(neighbors))
                else:
                    filtered.append(self.max_range)
            else:
                filtered.append(current)
        
        return filtered
    
    def detect_obstacles(self, ranges: List[float], angles: List[float],
                        min_distance: float = 0.5) -> List[Tuple[float, float]]:
        """
        Detecta obstáculos en los datos LiDAR.
        
        Args:
            ranges: Distancias medidas
            angles: Ángulos correspondientes
            min_distance: Distancia mínima para considerar obstáculo
            
        Returns:
            Lista de obstáculos (ángulo, distancia)
        """
        obstacles = []
        
        for range_val, angle in zip(ranges, angles):
            if 0 < range_val < min_distance:
                obstacles.append((angle, range_val))
        
        return obstacles
    
    def find_gaps(self, ranges: List[float], angles: List[float],
                 gap_threshold: float = 1.0, min_gap_width: float = 0.3) -> List[Tuple[float, float]]:
        """
        Encuentra espacios libres en los datos LiDAR.
        
        Args:
            ranges: Distancias medidas
            angles: Ángulos correspondientes
            gap_threshold: Distancia mínima para considerar espacio libre
            min_gap_width: Ancho mínimo del espacio en radianes
            
        Returns:
            Lista de espacios libres (ángulo_inicio, ángulo_fin)
        """
        gaps = []
        in_gap = False
        gap_start = 0
        
        for i, (range_val, angle) in enumerate(zip(ranges, angles)):
            is_free = range_val > gap_threshold
            
            if is_free and not in_gap:
                # Inicio de un nuevo espacio
                in_gap = True
                gap_start = angle
            elif not is_free and in_gap:
                # Fin del espacio actual
                gap_width = abs(angle - gap_start)
                if gap_width >= min_gap_width:
                    gaps.append((gap_start, angle))
                in_gap = False
        
        # Verificar si hay un espacio al final
        if in_gap and len(angles) > 0:
            gap_width = abs(angles[-1] - gap_start)
            if gap_width >= min_gap_width:
                gaps.append((gap_start, angles[-1]))
        
        return gaps
    
    def get_closest_obstacle(self, ranges: List[float], 
                           angles: List[float]) -> Optional[Tuple[float, float]]:
        """
        Encuentra el obstáculo más cercano.
        
        Args:
            ranges: Distancias medidas
            angles: Ángulos correspondientes
            
        Returns:
            Tupla (ángulo, distancia) del obstáculo más cercano, o None
        """
        min_distance = float('inf')
        closest_obstacle = None
        
        for range_val, angle in zip(ranges, angles):
            if 0 < range_val < min_distance:
                min_distance = range_val
                closest_obstacle = (angle, range_val)
        
        return closest_obstacle
    
    def convert_to_cartesian(self, ranges: List[float], 
                           angles: List[float]) -> List[Tuple[float, float]]:
        """
        Convierte datos polares a coordenadas cartesianas.
        
        Args:
            ranges: Distancias
            angles: Ángulos en radianes
            
        Returns:
            Lista de puntos (x, y)
        """
        points = []
        
        for range_val, angle in zip(ranges, angles):
            if range_val > 0:
                x = range_val * math.cos(angle)
                y = range_val * math.sin(angle)
                points.append((x, y))
        
        return points
    
    def calculate_front_clearance(self, ranges: List[float], angles: List[float],
                                front_angle_range: float = math.pi/6) -> float:
        """
        Calcula la distancia libre frontal promedio.
        
        Args:
            ranges: Distancias medidas
            angles: Ángulos correspondientes
            front_angle_range: Rango angular frontal a considerar
            
        Returns:
            Distancia libre frontal promedio
        """
        front_distances = []
        
        for range_val, angle in zip(ranges, angles):
            if abs(angle) <= front_angle_range and range_val > 0:
                front_distances.append(range_val)
        
        if not front_distances:
            return self.max_range
        
        return sum(front_distances) / len(front_distances)
