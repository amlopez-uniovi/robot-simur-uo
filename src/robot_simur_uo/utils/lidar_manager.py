# Clase para manejar dispositivos LiDAR en Webots
# Proporciona una interfaz unificada para gestión de LiDAR y procesamiento de datos

import math
import numpy as np
from typing import List, Tuple, Optional, Dict, Any
try:
    from controller import Robot
except ImportError:
    # Si no se encuentra el módulo `controller`, define un stub
    class Robot:
        def __init__(self, *args, **kwargs):
            raise RuntimeError("El módulo `controller` solo está disponible en el entorno de Webots.")


class LidarManager:
    """
    Clase para manejar dispositivos LiDAR en Webots.
    
    Proporciona funcionalidades para:
    - Inicialización y configuración del LiDAR
    - Obtención y procesamiento de datos
    - Análisis de lecturas (obstáculos, distancias, etc.)
    - Filtrado y validación de datos
    """
    
    def __init__(self, robot: Robot, device_name: str = "lidar", 
                 sweep_range: Tuple[float, float] = (0.0, 2*math.pi), 
                 time_step: int = 32):
        """
        Inicializar el gestor de LiDAR.
        
        Args:
            robot: Instancia del robot de Webots
            device_name: Nombre del dispositivo LiDAR en Webots
            sweep_range: Rango de barrido (ángulo_inicio, ángulo_fin) en radianes
            time_step: Paso de tiempo para habilitar el dispositivo
        """
        self.robot = robot
        self.device_name = device_name
        self.sweep_range = sweep_range
        self.time_step = time_step
                
        # Inicializar dispositivo LiDAR
        self.lidar_device = None
        self._initialize_lidar()
        
        # Configuración del LiDAR
        self.range_count = 0
        self.min_range = 0.0
        self.max_range = 0.0
        self.fov = 0.0
        self.angular_resolution = 0.0
        
        # Datos actuales
        self.current_data: List[float] = []
        self.filtered_data: List[float] = []
        self.valid_readings_count = 0
        self.invalid_readings_count = 0
        
        # Estadísticas
        self.stats: Dict[str, Any] = {}
        
        # Obtener configuración si el LiDAR está disponible
        if self.lidar_device:
            self._update_configuration()
    
    def set_sweep_range(self, sweep_range: Tuple[float, float]) -> None:
        """Establecer el rango de barrido del LiDAR."""
        self.sweep_range = sweep_range
        # Recalcular ángulos si ya tenemos configuración
        if hasattr(self, 'range_count') and self.range_count > 1:
            self.angles = np.linspace(self.sweep_range[0], self.sweep_range[1], self.range_count)

    def _initialize_lidar(self) -> bool:
        """
        Inicializar el dispositivo LiDAR.
        
        Returns:
            bool: True si se inicializó correctamente, False en caso contrario
        """
        try:
            self.lidar_device = self.robot.getDevice(self.device_name)
            if self.lidar_device is not None:
                self.lidar_device.enable(self.time_step)
                self.lidar_device.enablePointCloud()
                print(f"✅ LiDAR '{self.device_name}' inicializado correctamente")
                return True
            else:
                print(f"❌ Error: No se encontró el dispositivo LiDAR '{self.device_name}'")
                return False
        except Exception as e:
            print(f"❌ Error inicializando LiDAR '{self.device_name}': {e}")
            self.lidar_device = None
            return False
    
    def _update_configuration(self) -> None:
        """Actualizar la configuración del LiDAR desde el dispositivo."""
        if not self.lidar_device:
            return
        
        try:
            self.range_count = self.lidar_device.getNumberOfPoints()
            self.min_range = self.lidar_device.getMinRange()
            self.max_range = self.lidar_device.getMaxRange()
            self.fov = self.lidar_device.getFov()
            
            # Calcular resolución angular
            if self.range_count > 1:
                self.angular_resolution = self.fov / (self.range_count - 1)
                self.angles = np.linspace(self.sweep_range[0], self.sweep_range[1], self.range_count)
            else:
                self.angular_resolution = 0.0
                self.angles = []
                
        except Exception as e:
            print(f"⚠️ Error obteniendo configuración del LiDAR: {e}")
    
    def is_available(self) -> bool:
        """
        Verificar si el LiDAR está disponible y funcionando.
        
        Returns:
            bool: True si está disponible, False en caso contrario
        """
        return self.lidar_device is not None
    
    def get_raw_data(self) -> List[float]:
        """
        Obtener datos crudos del LiDAR.
        
        Returns:
            List[float]: Lista de distancias en metros, lista vacía si hay error
        """
        if not self.lidar_device:
            return []
        
        try:
            range_image = self.lidar_device.getRangeImage()
            if range_image:
                self.current_data = list(range_image)
                return self.current_data
            else:
                return []
        except Exception as e:
            print(f"⚠️ Error obteniendo datos del LiDAR: {e}")
            return []
    
    def get_filtered_data(self, min_valid: float = None, max_valid: float = None,
                         filter_inf: bool = True, filter_zero: bool = True) -> List[float]:
        """
        Obtener datos filtrados del LiDAR.
        
        Args:
            min_valid: Distancia mínima válida (usa min_range del LiDAR si es None)
            max_valid: Distancia máxima válida (usa max_range del LiDAR si es None)
            filter_inf: Si filtrar valores infinitos
            filter_zero: Si filtrar valores cero
            
        Returns:
            List[float]: Lista de distancias filtradas
        """
        raw_data = self.get_raw_data()
        if not raw_data:
            return []
        
        # Usar rangos del LiDAR si no se especifican
        min_val = min_valid if min_valid is not None else self.min_range
        max_val = max_valid if max_valid is not None else self.max_range
        
        filtered = []
        valid_count = 0
        invalid_count = 0
        
        for distance in raw_data:
            is_valid = True
            
            # Filtrar infinitos
            if filter_inf and math.isinf(distance):
                is_valid = False
            
            # Filtrar ceros
            if filter_zero and distance == 0.0:
                is_valid = False
            
            # Filtrar fuera de rango
            if is_valid and (distance < min_val or distance > max_val):
                is_valid = False
            
            if is_valid:
                filtered.append(distance)
                valid_count += 1
            else:
                invalid_count += 1
        
        self.filtered_data = filtered
        self.valid_readings_count = valid_count
        self.invalid_readings_count = invalid_count
        
        return filtered
    
    def get_raw_data_with_angles(self) -> List[Tuple[float, float]]:
        """
        Obtener datos crudos del LiDAR con sus ángulos correspondientes.
        
        Returns:
            List[Tuple[float, float]]: Lista de (distancia, ángulo) en metros y radianes
        """
        raw_data = self.get_raw_data()
        if not raw_data or not hasattr(self, 'angles') or len(self.angles) == 0:
            return []
        
        data_with_angles = []
        for i, distance in enumerate(raw_data):
            if i < len(self.angles):
                angle = self.angles[i]
                data_with_angles.append((distance, angle))
        
        return data_with_angles
    
    def get_angle_for_index(self, index: int) -> float:
        """
        Obtener el ángulo correspondiente a un índice específico.
        
        Args:
            index: Índice del punto LiDAR
            
        Returns:
            float: Ángulo en radianes
        """
        if self.range_count <= 1:
            return 0.0
        
        if not hasattr(self, 'angles') or len(self.angles) == 0:
            return 0.0
            
        if index >= len(self.angles):
            return 0.0
        
        # Usar ángulos pre-calculados
        return self.angles[index]
    
    def get_cartesian_points(self, use_filtered: bool = True) -> List[Tuple[float, float]]:
        """
        Convertir datos LiDAR a coordenadas cartesianas (x, y).
        
        Args:
            use_filtered: Si usar datos filtrados o crudos
            
        Returns:
            List[Tuple[float, float]]: Lista de puntos (x, y) en metros
        """
        data = self.filtered_data if use_filtered else self.current_data
        if not data:
            return []
        
        points = []
        for i, distance in enumerate(data):
            if not math.isinf(distance) and distance > 0:
                angle = self.get_angle_for_index(i)
                x = distance * math.cos(angle)
                y = distance * math.sin(angle)
                points.append((x, y))
        
        return points
    
    def find_obstacles(self, threshold: float = 1.0) -> List[Tuple[int, float, float]]:
        """
        Encontrar obstáculos dentro de un umbral de distancia.
        
        Args:
            threshold: Distancia máxima para considerar como obstáculo
            
        Returns:
            List[Tuple[int, float, float]]: Lista de (índice, distancia, ángulo)
        """
        raw_data = self.get_raw_data()
        if not raw_data:
            return []
        
        obstacles = []
        for i, distance in enumerate(raw_data):
            if not math.isinf(distance) and 0 < distance <= threshold:
                angle = self.get_angle_for_index(i)
                obstacles.append((i, distance, angle))
        
        return obstacles
    
    def get_statistics(self) -> Dict[str, Any]:
        """
        Obtener estadísticas de los datos LiDAR actuales.
        
        Returns:
            Dict[str, Any]: Diccionario con estadísticas
        """
        raw_data = self.current_data
        filtered_data = self.filtered_data
        
        if not raw_data:
            return {}
        
        # Estadísticas básicas
        total_points = len(raw_data)
        inf_count = sum(1 for x in raw_data if math.isinf(x))
        zero_count = sum(1 for x in raw_data if x == 0.0)
        valid_points = [x for x in raw_data if not math.isinf(x) and x > 0]
        
        stats = {
            'total_points': total_points,
            'valid_points': len(valid_points),
            'infinite_points': inf_count,
            'zero_points': zero_count,
            'invalid_points': self.invalid_readings_count,
            'filtered_points': len(filtered_data),
        }
        
        # Estadísticas de distancias válidas
        if valid_points:
            stats.update({
                'min_distance': min(valid_points),
                'max_distance': max(valid_points),
                'avg_distance': sum(valid_points) / len(valid_points),
                'median_distance': sorted(valid_points)[len(valid_points)//2]
            })
        
        self.stats = stats
        return stats
    
    def get_configuration_info(self) -> Dict[str, Any]:
        """
        Obtener información de configuración del LiDAR.
        
        Returns:
            Dict[str, Any]: Información de configuración
        """
        return {
            'device_name': self.device_name,
            'range_count': self.range_count,
            'min_range': self.min_range,
            'max_range': self.max_range,
            'fov': self.fov,
            'angular_resolution': self.angular_resolution,
            'sweep_range': self.sweep_range,
            'time_step': self.time_step,
            'is_available': self.is_available()
        }
    
    def print_summary(self) -> None:
        """Imprimir un resumen del estado actual del LiDAR."""
        if not self.is_available():
            print(f"❌ LiDAR '{self.device_name}' no está disponible")
            return
        
        config = self.get_configuration_info()
        stats = self.get_statistics()
        
        print(f"\n📡 === LiDAR Manager: {self.device_name} ===")
        print(f"🔧 Configuración:")
        print(f"   Puntos de medición: {config['range_count']}")
        print(f"   Rango: {config['min_range']:.3f}m - {config['max_range']:.3f}m")
        print(f"   Campo de visión: {math.degrees(config['fov']):.1f}°")
        print(f"   Resolución angular: {math.degrees(config['angular_resolution']):.2f}°")
        
        # Imprimir todos los datos LiDAR con ángulos
        raw_data_with_angles = self.get_raw_data_with_angles()
        if raw_data_with_angles:
            print(f"📊 Datos LiDAR completos (ángulo, distancia):")
            for i, (distance, angle) in enumerate(raw_data_with_angles):
                if math.isinf(distance):
                    print(f"   [{i:3d}]: ({math.degrees(angle):6.1f}°, inf)")
                else:
                    print(f"   [{i:3d}]: ({math.degrees(angle):6.1f}°, {distance:6.3f}m)")
        
        if stats:
            print(f"📈 Estadísticas:")
            print(f"   Total de puntos: {stats['total_points']}")
            print(f"   Puntos válidos: {stats['valid_points']}")
            print(f"   Puntos infinitos: {stats['infinite_points']}")
            print(f"   Puntos cero: {stats['zero_points']}")
            
            if 'min_distance' in stats:
                print(f"   Distancia min/max: {stats['min_distance']:.3f}m / {stats['max_distance']:.3f}m")
                print(f"   Distancia promedio: {stats['avg_distance']:.3f}m")
        
        print("=" * 50)
    
    def print_all_lidar_data(self) -> None:
        """Imprimir todos los datos LiDAR con sus ángulos correspondientes."""
        if not self.is_available():
            print(f"❌ LiDAR '{self.device_name}' no está disponible")
            return
        
        try:
            raw_data_with_angles = self.get_raw_data_with_angles()
            if not raw_data_with_angles:
                print("📊 No hay datos LiDAR disponibles")
                return
            
            print(f"\n📊 === Datos LiDAR Completos: {self.device_name} ===")
            print(f"Total de puntos: {len(raw_data_with_angles)}")
            print("Formato: [índice]: (ángulo°, distancia)")
            print("-" * 40)
            
            for i, (distance, angle) in enumerate(raw_data_with_angles):
                if math.isinf(distance):
                    print(f"[{i:3d}]: ({math.degrees(angle):6.1f}°, inf)")
                else:
                    print(f"[{i:3d}]: ({math.degrees(angle):6.1f}°, {distance:6.3f}m)")
            
            print("=" * 50)
            
        except Exception as e:
            print(f"⚠️ Error al imprimir datos LiDAR: {e}")
