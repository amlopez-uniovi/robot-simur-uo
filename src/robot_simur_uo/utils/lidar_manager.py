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
    def get_closest_obstacle_in_angle_range(self, angle_min: float, angle_max: float):
        """
        Obtener información de obstáculos en un rango de ángulos.
        Args:
            angle_min (float): Ángulo mínimo en radianes (inicio del sector)
            angle_max (float): Ángulo máximo en radianes (fin del sector)
        Returns:
            Tuple[List[float], List[float], float, float]:
                - Lista de ángulos dentro del rango
                - Lista de distancias correspondientes
                - Ángulo del obstáculo más cercano
                - Distancia del obstáculo más cercano
                Si no hay datos válidos, listas vacías y float('inf')
        """
        try:
            data_with_angles = self.get_raw_data_with_angles()
            if not data_with_angles:
                return [], [], float('inf'), float('inf')
            # Filtrar puntos dentro del rango de ángulos
            filtered = [(d, a) for d, a in data_with_angles if angle_min <= a <= angle_max and d > 0 and not math.isinf(d)]
            if not filtered:
                return [], [], float('inf'), float('inf')
            angles = [a for d, a in filtered]
            distances = [d for d, a in filtered]
            min_idx = distances.index(min(distances))
            min_angle = angles[min_idx]
            min_distance = distances[min_idx]
            return angles, distances, min_angle, min_distance
        except Exception as e:
            print(f"⚠️ Error calculando obstáculo en rango [{math.degrees(angle_min):.1f}°, {math.degrees(angle_max):.1f}°]: {e}")
            return [], [], float('inf'), float('inf')
    """
    Clase para manejar dispositivos LiDAR en Webots.
    
    Proporciona funcionalidades para:
    - Inicialización y configuración del LiDAR
    - Obtención y procesamiento de datos
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
    
    def get_raw_data_with_angles(self) -> List[Tuple[float, float]]:
        """
        Obtener datos crudos del LiDAR con sus ángulos correspondientes.
        
        Returns:
            List[Tuple[float, float]]: Lista de (distancia, ángulo) en metros y radianes
        """
        raw_data = self.get_raw_data()
        if not raw_data or not hasattr(self, 'angles') or len(self.angles) == 0:
            return []
        
        # crear pares (distancia, ángulo) 
        min_length = min(len(raw_data), len(self.angles))
        data_with_angles = list(zip(raw_data[:min_length], self.angles[:min_length]))
        
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
    
    def print_summary(self) -> str:
        """
        Generar un resumen del estado actual del LiDAR.
        
        Returns:
            str: String con el resumen del LiDAR
        """
        if not self.is_available():
            return f"❌ LiDAR '{self.device_name}' no está disponible"
        
        config = self.get_configuration_info()
        
        lines = []
        lines.append(f"\n📡 === LiDAR Manager: {self.device_name} ===")
        lines.append(f"🔧 Configuración:")
        lines.append(f"   Puntos de medición: {config['range_count']}")
        lines.append(f"   Rango: {config['min_range']:.3f}m - {config['max_range']:.3f}m")
        lines.append(f"   Campo de visión: {math.degrees(config['fov']):.1f}°")
        lines.append(f"   Resolución angular: {math.degrees(config['angular_resolution']):.2f}°")
        
        # Agregar datos LiDAR usando la función existente
        lidar_data = self.print_all_lidar_data()
        lines.append(lidar_data)
        
        return "\n".join(lines)
    
    def print_all_lidar_data(self) -> str:
        """
        Generar string con todos los datos LiDAR y sus ángulos correspondientes.
        
        Returns:
            str: String con todos los datos LiDAR formateados
        """
        if not self.is_available():
            return f"❌ LiDAR '{self.device_name}' no está disponible"
        
        try:
            raw_data_with_angles = self.get_raw_data_with_angles()
            if not raw_data_with_angles:
                return "📊 No hay datos LiDAR disponibles"
            
            lines = []
            lines.append(f"\n📊 === Datos LiDAR Completos: {self.device_name} ===")
            lines.append(f"Total de puntos: {len(raw_data_with_angles)}")
            lines.append("Formato: [índice]: (ángulo°, distancia)")
            lines.append("-" * 40)
            
            for i, (distance, angle) in enumerate(raw_data_with_angles):
                if math.isinf(distance):
                    lines.append(f"[{i:3d}]: ({math.degrees(angle):6.1f}°, inf)")
                else:
                    lines.append(f"[{i:3d}]: ({math.degrees(angle):6.1f}°, {distance:6.3f}m)")
            
            lines.append("=" * 50)
            
            return "\n".join(lines)
            
        except Exception as e:
            return f"⚠️ Error al generar datos LiDAR: {e}"
    
    def get_point_cloud(self) -> List:
        """
        Obtener nube de puntos del LiDAR.
        
        Returns:
            List: Nube de puntos, lista vacía si hay error
        """
        if not self.lidar_device:
            return []
        
        try:
            return self.lidar_device.getPointCloud()
        except Exception as e:
            print(f"⚠️ Error obteniendo nube de puntos: {e}")
            return []
    
    def get_range_count(self) -> int:
        """
        Obtener número de puntos del LiDAR.
        
        Returns:
            int: Número de puntos del LiDAR
        """
        return self.range_count
    
    def get_max_range(self) -> float:
        """
        Obtener rango máximo del LiDAR.
        
        Returns:
            float: Rango máximo en metros
        """
        return self.max_range
    
    def get_min_range(self) -> float:
        """
        Obtener rango mínimo del LiDAR.
        
        Returns:
            float: Rango mínimo en metros
        """
        return self.min_range
    
    def get_fov(self) -> float:
        """
        Obtener campo de visión del LiDAR.
        
        Returns:
            float: Campo de visión en radianes
        """
        return self.fov
    
    def get_closest_obstacle(self) -> float:
        """
        Obtener la distancia al obstáculo más cercano detectado por el LiDAR.
        
        Returns:
            float: Distancia al obstáculo más cercano en metros
        """
        try:
            ranges = self.get_raw_data()
            if ranges and len(ranges) > 0:
                # Filtrar valores infinitos o inválidos
                valid_ranges = [r for r in ranges if r != float('inf') and r > 0]
                if valid_ranges:
                    return min(valid_ranges)
        except Exception as e:
            print(f"⚠️ Error calculando obstáculo más cercano: {e}")
        
        return float('inf')
    
    def get_sectored_distances(self, num_sectors: int = 8, 
                             min_range: float = 0.05, 
                             max_range: float = 10.0) -> List[float]:
        """
        Divide los datos del LiDAR en sectores direccionales y obtiene la distancia mínima de cada sector.
        
        Args:
            num_sectors (int): Número de sectores en que dividir el LiDAR (8 por defecto)
            min_range (float): Distancia mínima válida en metros
            max_range (float): Distancia máxima válida en metros (valor por defecto para obstáculos lejanos)
            
        Returns:
            List[float]: Lista de distancias mínimas por sector
        """
        try:
            lidar_data = self.get_raw_data()
            
            if not lidar_data or len(lidar_data) == 0:
                return [max_range] * num_sectors
            
            num_points = len(lidar_data)
            sector_size = max(1, num_points // num_sectors)
            sectored_distances = []
            
            for i in range(num_sectors):
                # Calcular índices del sector
                start_idx = i * sector_size
                end_idx = min(start_idx + sector_size, num_points)
                
                # Obtener distancia mínima en este sector
                if start_idx < len(lidar_data):
                    sector_distances = lidar_data[start_idx:end_idx]
                    if sector_distances:
                        # Filtrar valores válidos en el sector
                        valid_distances = [
                            d for d in sector_distances 
                            if d > min_range and d < max_range and d != float('inf')
                        ]
                        
                        if valid_distances:
                            min_distance = min(valid_distances)
                            sectored_distances.append(min_distance)
                        else:
                            sectored_distances.append(max_range)  # Sin obstáculo válido
                    else:
                        sectored_distances.append(max_range)
                else:
                    sectored_distances.append(max_range)
            
            return sectored_distances
            
        except Exception as e:
            print(f"Error al procesar sectores del LiDAR: {e}")
            return [max_range] * num_sectors
    
    def get_obstacle_sensors(self, num_sectors: int = 8) -> List[float]:
        """
        Obtener sensores de obstáculos simples desde el LiDAR.
        Convierte datos del LiDAR en sectores para detección de obstáculos.
        
        Args:
            num_sectors (int): Número de sectores a crear (por defecto 8)
            
        Returns:
            List[float]: Lista con distancias mínimas por sector
        """
        lidar_data = self.get_raw_data()
        obstacle_sensors = []
        
        if lidar_data and len(lidar_data) > 0:
            # Convertir LiDAR a sectores simples
            num_points = len(lidar_data)
            sector_size = max(1, num_points // num_sectors)
            
            for i in range(num_sectors):
                start_idx = i * sector_size
                end_idx = min(start_idx + sector_size, num_points)
                
                if start_idx < len(lidar_data):
                    sector_distances = lidar_data[start_idx:end_idx]
                    if sector_distances:
                        min_distance = min(sector_distances)
                        # Filtrar valores inválidos (muy cerca o infinito)
                        obstacle_sensors.append(min_distance if min_distance > 0.02 else 1.0)
                    else:
                        obstacle_sensors.append(1.0)
                else:
                    obstacle_sensors.append(1.0)
        else:
            # Sin LiDAR, asumir sin obstáculos
            obstacle_sensors = [1.0] * num_sectors
        
        return obstacle_sensors
