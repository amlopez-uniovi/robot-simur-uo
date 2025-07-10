"""
Fusión de datos de múltiples sensores.
"""

import math
from typing import List, Dict, Tuple, Optional, Any


class SensorFusion:
    """Clase para fusionar datos de múltiples tipos de sensores."""
    
    def __init__(self, confidence_weights: Dict[str, float] = None):
        """
        Inicializa el sistema de fusión de sensores.
        
        Args:
            confidence_weights: Pesos de confianza para cada tipo de sensor
        """
        self.confidence_weights = confidence_weights or {
            'lidar': 0.8,
            'ultrasonic': 0.6,
            'camera': 0.5,
            'ir': 0.4
        }
        
        # Historial de mediciones para filtrado temporal
        self.measurement_history = {}
        self.max_history_size = 10
    
    def add_measurement(self, sensor_type: str, measurement: Any, timestamp: float = None):
        """
        Añade una medición al historial.
        
        Args:
            sensor_type: Tipo de sensor
            measurement: Medición del sensor
            timestamp: Marca temporal (se genera automáticamente si es None)
        """
        if timestamp is None:
            import time
            timestamp = time.time()
        
        if sensor_type not in self.measurement_history:
            self.measurement_history[sensor_type] = []
        
        self.measurement_history[sensor_type].append({
            'measurement': measurement,
            'timestamp': timestamp
        })
        
        # Mantener solo las mediciones más recientes
        if len(self.measurement_history[sensor_type]) > self.max_history_size:
            self.measurement_history[sensor_type].pop(0)
    
    def fuse_distance_measurements(self, measurements: Dict[str, float]) -> float:
        """
        Fusiona mediciones de distancia de múltiples sensores.
        
        Args:
            measurements: Diccionario {tipo_sensor: distancia}
            
        Returns:
            Distancia fusionada
        """
        if not measurements:
            return float('inf')
        
        weighted_sum = 0.0
        total_weight = 0.0
        
        for sensor_type, distance in measurements.items():
            if distance > 0:  # Filtrar mediciones inválidas
                weight = self.confidence_weights.get(sensor_type, 0.5)
                weighted_sum += distance * weight
                total_weight += weight
        
        if total_weight == 0:
            return float('inf')
        
        return weighted_sum / total_weight
    
    def fuse_obstacle_detection(self, detections: Dict[str, List[Tuple[float, float]]]) -> List[Tuple[float, float, float]]:
        """
        Fusiona detecciones de obstáculos de múltiples sensores.
        
        Args:
            detections: Diccionario {tipo_sensor: [(ángulo, distancia), ...]}
            
        Returns:
            Lista de obstáculos fusionados (ángulo, distancia, confianza)
        """
        all_obstacles = []
        
        # Recopilar todas las detecciones con sus pesos
        for sensor_type, obstacles in detections.items():
            weight = self.confidence_weights.get(sensor_type, 0.5)
            for angle, distance in obstacles:
                all_obstacles.append((angle, distance, weight))
        
        if not all_obstacles:
            return []
        
        # Agrupar obstáculos cercanos
        fused_obstacles = []
        angular_threshold = math.pi / 8  # 22.5 grados
        
        processed = [False] * len(all_obstacles)
        
        for i, (angle1, dist1, weight1) in enumerate(all_obstacles):
            if processed[i]:
                continue
            
            # Encontrar obstáculos similares
            similar_obstacles = [(angle1, dist1, weight1)]
            processed[i] = True
            
            for j, (angle2, dist2, weight2) in enumerate(all_obstacles):
                if processed[j]:
                    continue
                
                # Calcular diferencia angular
                angle_diff = abs(angle1 - angle2)
                angle_diff = min(angle_diff, 2*math.pi - angle_diff)
                
                if angle_diff < angular_threshold:
                    similar_obstacles.append((angle2, dist2, weight2))
                    processed[j] = True
            
            # Fusionar obstáculos similares
            if similar_obstacles:
                fused_obstacle = self._fuse_similar_obstacles(similar_obstacles)
                fused_obstacles.append(fused_obstacle)
        
        return fused_obstacles
    
    def _fuse_similar_obstacles(self, obstacles: List[Tuple[float, float, float]]) -> Tuple[float, float, float]:
        """
        Fusiona obstáculos similares en uno solo.
        
        Args:
            obstacles: Lista de obstáculos (ángulo, distancia, peso)
            
        Returns:
            Obstáculo fusionado (ángulo, distancia, confianza)
        """
        total_weight = sum(weight for _, _, weight in obstacles)
        
        # Promedio ponderado de ángulos (cuidado con la circularidad)
        sin_sum = sum(weight * math.sin(angle) for angle, _, weight in obstacles)
        cos_sum = sum(weight * math.cos(angle) for angle, _, weight in obstacles)
        fused_angle = math.atan2(sin_sum, cos_sum)
        
        # Promedio ponderado de distancias
        fused_distance = sum(dist * weight for _, dist, weight in obstacles) / total_weight
        
        # Confianza basada en número de sensores y pesos
        confidence = min(1.0, total_weight / len(obstacles))
        
        return fused_angle, fused_distance, confidence
    
    def estimate_robot_pose(self, odometry: Tuple[float, float, float],
                           landmark_observations: List[Tuple[float, float, float]]) -> Tuple[float, float, float]:
        """
        Estima la pose del robot fusionando odometría y observaciones de landmarks.
        
        Args:
            odometry: Pose estimada por odometría (x, y, theta)
            landmark_observations: Lista de observaciones (x_landmark, y_landmark, confianza)
            
        Returns:
            Pose fusionada (x, y, theta)
        """
        if not landmark_observations:
            return odometry
        
        odo_x, odo_y, odo_theta = odometry
        
        # Peso base para odometría
        odo_weight = 0.7
        
        # Calcular corrección basada en landmarks
        correction_x = 0.0
        correction_y = 0.0
        total_landmark_weight = 0.0
        
        for lm_x, lm_y, confidence in landmark_observations:
            weight = confidence * 0.3  # Peso máximo para landmarks
            correction_x += (lm_x - odo_x) * weight
            correction_y += (lm_y - odo_y) * weight
            total_landmark_weight += weight
        
        if total_landmark_weight > 0:
            correction_x /= total_landmark_weight
            correction_y /= total_landmark_weight
        
        # Aplicar corrección
        fused_x = odo_x + correction_x
        fused_y = odo_y + correction_y
        fused_theta = odo_theta  # Por simplicidad, mantenemos el ángulo de odometría
        
        return fused_x, fused_y, fused_theta
    
    def temporal_filter(self, sensor_type: str, current_value: float,
                       filter_strength: float = 0.3) -> float:
        """
        Aplica filtro temporal a una medición.
        
        Args:
            sensor_type: Tipo de sensor
            current_value: Valor actual
            filter_strength: Fuerza del filtro (0-1)
            
        Returns:
            Valor filtrado
        """
        if sensor_type not in self.measurement_history:
            return current_value
        
        history = self.measurement_history[sensor_type]
        if not history:
            return current_value
        
        # Promedio de valores recientes
        recent_values = [entry['measurement'] for entry in history[-3:]]
        if isinstance(current_value, (int, float)):
            avg_recent = sum(recent_values) / len(recent_values)
            return current_value * (1 - filter_strength) + avg_recent * filter_strength
        
        return current_value
    
    def get_sensor_health(self, sensor_type: str, 
                         max_age: float = 5.0) -> Dict[str, Any]:
        """
        Evalúa el estado de salud de un sensor.
        
        Args:
            sensor_type: Tipo de sensor
            max_age: Edad máxima aceptable de mediciones en segundos
            
        Returns:
            Diccionario con información de salud del sensor
        """
        import time
        current_time = time.time()
        
        health_info = {
            'is_active': False,
            'last_update': None,
            'age': float('inf'),
            'measurement_rate': 0.0,
            'status': 'unknown'
        }
        
        if sensor_type not in self.measurement_history:
            health_info['status'] = 'no_data'
            return health_info
        
        history = self.measurement_history[sensor_type]
        if not history:
            health_info['status'] = 'no_data'
            return health_info
        
        # Última medición
        last_measurement = history[-1]
        last_time = last_measurement['timestamp']
        age = current_time - last_time
        
        health_info['last_update'] = last_time
        health_info['age'] = age
        health_info['is_active'] = age < max_age
        
        # Calcular tasa de mediciones
        if len(history) > 1:
            time_span = history[-1]['timestamp'] - history[0]['timestamp']
            if time_span > 0:
                health_info['measurement_rate'] = len(history) / time_span
        
        # Determinar estado
        if age > max_age:
            health_info['status'] = 'stale'
        elif health_info['measurement_rate'] < 0.1:
            health_info['status'] = 'low_rate'
        else:
            health_info['status'] = 'healthy'
        
        return health_info
