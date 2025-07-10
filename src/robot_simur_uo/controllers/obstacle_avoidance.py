"""
Controlador para evitación de obstáculos.
"""

import math
from typing import List, Tuple


class ObstacleAvoidanceController:
    """Controlador para evitar obstáculos usando sensores de distancia."""
    
    def __init__(self, safe_distance: float = 0.15, avoidance_gain: float = 1.0):
        """
        Inicializa el controlador de evitación de obstáculos.
        
        Args:
            safe_distance: Distancia mínima segura a obstáculos
            avoidance_gain: Ganancia para la respuesta de evitación
        """
        self.safe_distance = safe_distance
        self.avoidance_gain = avoidance_gain
    
    def detect_obstacles(self, sensor_values: List[float], 
                        sensor_angles: List[float]) -> List[Tuple[float, float]]:
        """
        Detecta obstáculos basándose en los valores de sensores.
        
        Args:
            sensor_values: Lista de valores de sensores de distancia
            sensor_angles: Lista de ángulos de los sensores (en radianes)
            
        Returns:
            Lista de tuplas (ángulo, distancia) de obstáculos detectados
        """
        obstacles = []
        
        for i, (distance, angle) in enumerate(zip(sensor_values, sensor_angles)):
            if distance < self.safe_distance:
                obstacles.append((angle, distance))
        
        return obstacles
    
    def calculate_avoidance_force(self, obstacles: List[Tuple[float, float]]) -> Tuple[float, float]:
        """
        Calcula la fuerza de evitación resultante.
        
        Args:
            obstacles: Lista de obstáculos (ángulo, distancia)
            
        Returns:
            Tuple con (fuerza_x, fuerza_y) de evitación
        """
        total_force_x = 0.0
        total_force_y = 0.0
        
        for angle, distance in obstacles:
            if distance > 0:
                # Fuerza inversamente proporcional a la distancia
                force_magnitude = self.avoidance_gain / (distance + 0.01)
                
                # Fuerza en dirección opuesta al obstáculo
                force_x = -force_magnitude * math.cos(angle)
                force_y = -force_magnitude * math.sin(angle)
                
                total_force_x += force_x
                total_force_y += force_y
        
        return total_force_x, total_force_y
    
    def calculate_avoidance_speeds(self, sensor_values: List[float], 
                                 sensor_angles: List[float],
                                 base_speed: float = 0.5) -> Tuple[float, float]:
        """
        Calcula velocidades de motores con evitación de obstáculos.
        
        Args:
            sensor_values: Valores de sensores de distancia
            sensor_angles: Ángulos de sensores
            base_speed: Velocidad base deseada
            
        Returns:
            Tuple con (velocidad_izquierda, velocidad_derecha)
        """
        obstacles = self.detect_obstacles(sensor_values, sensor_angles)
        
        if not obstacles:
            # No hay obstáculos, avanzar normalmente
            return base_speed, base_speed
        
        # Calcular fuerza de evitación
        force_x, force_y = self.calculate_avoidance_force(obstacles)
        
        # Convertir fuerza a ajuste angular
        avoidance_angle = math.atan2(force_y, force_x)
        
        # Calcular ajuste de velocidades
        angular_adjustment = avoidance_angle * 0.5
        
        left_speed = base_speed - angular_adjustment
        right_speed = base_speed + angular_adjustment
        
        # Reducir velocidad si hay obstáculos muy cerca
        min_distance = min([dist for _, dist in obstacles])
        speed_reduction = max(0.1, min_distance / self.safe_distance)
        
        left_speed *= speed_reduction
        right_speed *= speed_reduction
        
        return left_speed, right_speed
    
    def is_path_clear(self, sensor_values: List[float], 
                     front_sensor_indices: List[int] = None) -> bool:
        """
        Verifica si el camino frontal está despejado.
        
        Args:
            sensor_values: Valores de sensores
            front_sensor_indices: Índices de sensores frontales
            
        Returns:
            True si el camino está despejado
        """
        if front_sensor_indices is None:
            # Usar sensores centrales por defecto
            num_sensors = len(sensor_values)
            front_sensor_indices = [num_sensors // 2 - 1, num_sensors // 2]
        
        for idx in front_sensor_indices:
            if idx < len(sensor_values) and sensor_values[idx] < self.safe_distance:
                return False
        
        return True
