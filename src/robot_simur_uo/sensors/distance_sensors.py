"""
Procesador para sensores de distancia.
"""

import math
from typing import List, Tuple, Optional


class DistanceSensorProcessor:
    """Procesador para sensores de distancia ultrasónicos o infrarrojos."""
    
    def __init__(self, num_sensors: int = 8, max_range: float = 1.0):
        """
        Inicializa el procesador de sensores de distancia.
        
        Args:
            num_sensors: Número de sensores
            max_range: Rango máximo de los sensores en metros
        """
        self.num_sensors = num_sensors
        self.max_range = max_range
        
        # Ángulos de sensores distribuidos uniformemente
        self.sensor_angles = [
            (2 * math.pi * i / num_sensors) - math.pi 
            for i in range(num_sensors)
        ]
    
    def filter_readings(self, readings: List[float], 
                       filter_type: str = "median") -> List[float]:
        """
        Filtra las lecturas de sensores para reducir ruido.
        
        Args:
            readings: Lista de lecturas de sensores
            filter_type: Tipo de filtro ("median", "average", "none")
            
        Returns:
            Lista de lecturas filtradas
        """
        if filter_type == "none" or len(readings) < 3:
            return readings
        
        filtered = []
        
        for i in range(len(readings)):
            if filter_type == "median":
                # Filtro de mediana con ventana de 3
                window = []
                for j in range(max(0, i-1), min(len(readings), i+2)):
                    window.append(readings[j])
                window.sort()
                filtered.append(window[len(window)//2])
            
            elif filter_type == "average":
                # Filtro de promedio con ventana de 3
                window_sum = 0
                window_count = 0
                for j in range(max(0, i-1), min(len(readings), i+2)):
                    window_sum += readings[j]
                    window_count += 1
                filtered.append(window_sum / window_count)
        
        return filtered
    
    def detect_obstacles_by_sector(self, readings: List[float],
                                  obstacle_threshold: float = 0.3) -> dict:
        """
        Detecta obstáculos por sectores (frontal, lateral izquierdo, etc.).
        
        Args:
            readings: Lecturas de sensores
            obstacle_threshold: Distancia mínima para considerar obstáculo
            
        Returns:
            Diccionario con sectores y distancia mínima en cada uno
        """
        sectors = {
            'front': float('inf'),
            'front_left': float('inf'),
            'front_right': float('inf'),
            'left': float('inf'),
            'right': float('inf'),
            'back_left': float('inf'),
            'back_right': float('inf'),
            'back': float('inf')
        }
        
        if len(readings) != self.num_sensors:
            return sectors
        
        for i, (reading, angle) in enumerate(zip(readings, self.sensor_angles)):
            if reading <= 0:
                continue
            
            # Determinar sector basado en ángulo
            angle_deg = math.degrees(angle)
            
            if -22.5 <= angle_deg <= 22.5:
                sector = 'front'
            elif 22.5 < angle_deg <= 67.5:
                sector = 'front_left'
            elif 67.5 < angle_deg <= 112.5:
                sector = 'left'
            elif 112.5 < angle_deg <= 157.5:
                sector = 'back_left'
            elif 157.5 < angle_deg or angle_deg <= -157.5:
                sector = 'back'
            elif -157.5 < angle_deg <= -112.5:
                sector = 'back_right'
            elif -112.5 < angle_deg <= -67.5:
                sector = 'right'
            elif -67.5 < angle_deg <= -22.5:
                sector = 'front_right'
            else:
                continue
            
            # Actualizar distancia mínima del sector
            sectors[sector] = min(sectors[sector], reading)
        
        return sectors
    
    def get_front_clearance(self, readings: List[float],
                          front_sensors: List[int] = None) -> float:
        """
        Obtiene la distancia libre frontal.
        
        Args:
            readings: Lecturas de sensores
            front_sensors: Índices de sensores frontales
            
        Returns:
            Distancia libre frontal mínima
        """
        if front_sensors is None:
            # Usar sensores centrales por defecto
            center = self.num_sensors // 2
            front_sensors = [center - 1, center] if center > 0 else [0]
        
        min_distance = self.max_range
        
        for sensor_idx in front_sensors:
            if 0 <= sensor_idx < len(readings) and readings[sensor_idx] > 0:
                min_distance = min(min_distance, readings[sensor_idx])
        
        return min_distance
    
    def calculate_obstacle_vector(self, readings: List[float]) -> Tuple[float, float]:
        """
        Calcula un vector resultante hacia obstáculos cercanos.
        
        Args:
            readings: Lecturas de sensores
            
        Returns:
            Tupla (x, y) del vector hacia obstáculos
        """
        vector_x = 0.0
        vector_y = 0.0
        
        for reading, angle in zip(readings, self.sensor_angles):
            if reading > 0 and reading < self.max_range:
                # Peso inversamente proporcional a la distancia
                weight = 1.0 / (reading + 0.1)
                
                # Vector hacia el obstáculo
                vector_x += weight * math.cos(angle)
                vector_y += weight * math.sin(angle)
        
        return vector_x, vector_y
    
    def find_best_direction(self, readings: List[float],
                          num_directions: int = 8) -> float:
        """
        Encuentra la mejor dirección para evitar obstáculos.
        
        Args:
            readings: Lecturas de sensores
            num_directions: Número de direcciones a evaluar
            
        Returns:
            Ángulo en radianes de la mejor dirección
        """
        best_angle = 0.0
        best_clearance = 0.0
        
        for i in range(num_directions):
            angle = (2 * math.pi * i / num_directions) - math.pi
            
            # Calcular distancia promedio en esa dirección
            clearance = self._calculate_clearance_in_direction(readings, angle)
            
            if clearance > best_clearance:
                best_clearance = clearance
                best_angle = angle
        
        return best_angle
    
    def _calculate_clearance_in_direction(self, readings: List[float], 
                                        direction: float) -> float:
        """
        Calcula la distancia libre en una dirección específica.
        
        Args:
            readings: Lecturas de sensores
            direction: Dirección en radianes
            
        Returns:
            Distancia libre en esa dirección
        """
        clearance = self.max_range
        angular_tolerance = math.pi / 4  # 45 grados
        
        for reading, sensor_angle in zip(readings, self.sensor_angles):
            if reading <= 0:
                continue
            
            # Calcular diferencia angular
            angle_diff = abs(direction - sensor_angle)
            angle_diff = min(angle_diff, 2*math.pi - angle_diff)  # Menor diferencia
            
            if angle_diff <= angular_tolerance:
                # Ponderar por proximidad angular
                weight = 1.0 - (angle_diff / angular_tolerance)
                weighted_distance = reading * weight
                clearance = min(clearance, weighted_distance)
        
        return clearance
    
    def is_safe_to_move(self, readings: List[float], 
                       safety_distance: float = 0.2) -> bool:
        """
        Verifica si es seguro moverse hacia adelante.
        
        Args:
            readings: Lecturas de sensores
            safety_distance: Distancia mínima de seguridad
            
        Returns:
            True si es seguro moverse
        """
        front_clearance = self.get_front_clearance(readings)
        return front_clearance > safety_distance
