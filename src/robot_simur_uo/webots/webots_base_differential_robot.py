try:
    from controller import Robot
except ImportError:
    # Si no se encuentra el módulo `controller`, define un stub o lanza una advertencia
    class Robot:
        def __init__(self, *args, **kwargs):
            raise RuntimeError("El módulo `controller` solo está disponible en el entorno de Webots.")

import math
from typing import Tuple
from ..interfaces.idifferential_robot import IDifferentialRobot
from ..utils.coordinates import RobotPose

class WebotsBaseDifferentialRobot(IDifferentialRobot):
    """Clase base para robots diferenciales en Webots con funcionalidades comunes"""
    
    def __init__(self, time_step=64):
        """Inicializar el robot base y sus componentes comunes"""
        self.robot = Robot()
        self.time_step = time_step
        
        # Inicializar componentes comunes
        self._init_common_components()
        
        # Inicializar componentes específicos del robot (implementado en subclases)
        self._init_specific_components()
    
    def _init_common_components(self):
        """Inicializar componentes comunes a todos los robots"""
        self._init_navigation_sensors()
        self._init_lidar()
    
    def _init_specific_components(self):
        """Inicializar componentes específicos del robot (debe ser implementado por subclases)"""
        raise NotImplementedError("Subclases deben implementar _init_specific_components")
    
    def _init_navigation_sensors(self):
        """Inicializar GPS y brújula para navegación"""
        self.gps_sensor = self.robot.getDevice("gps")
        self.gps_sensor.enable(self.time_step)
        
        self.compass_sensor = self.robot.getDevice("compass")
        self.compass_sensor.enable(self.time_step)
    
    def _init_lidar(self):
        """Inicializar lidar/laser común a todos los robots"""
        self.lidar_sensor = self.robot.getDevice("laser")
        self.lidar_sensor.enable(self.time_step)
        self.lidar_sensor.enablePointCloud()
    
    def step(self, time_step=None):
        """Ejecutar un paso de simulación
        
        Args:
            time_step (int, optional): Duración del paso en milisegundos. 
                                      Si no se especifica, usa self.time_step
        
        Returns:
            int: 0 si la simulación continúa, -1 si debe terminar
        """
        if time_step is None:
            time_step = self.time_step
        return self.robot.step(time_step)
    
    def get_gps_position(self):
        """Obtener posición GPS"""
        return self.gps_sensor.getValues()
    
    def get_compass_orientation(self):
        """Obtener orientación de la brújula"""
        direction = self.compass_sensor.getValues()
        angle = math.pi / 2 -math.atan2(direction[1], direction[0])
        return direction, angle
    
    def get_lidar_data(self):
        """Obtener datos del lidar/laser"""
        return self.lidar_sensor.getRangeImage()
    
    def get_lidar_point_cloud(self):
        """Obtener nube de puntos del lidar"""
        try:
            return self.lidar_sensor.getPointCloud()
        except:
            return []
    
    def get_lidar_range_count(self):
        """Obtener número de puntos del lidar"""
        try:
            return self.lidar_sensor.getNumberOfPoints()
        except:
            return 0
    
    def get_lidar_max_range(self):
        """Obtener rango máximo del lidar"""
        try:
            return self.lidar_sensor.getMaxRange()
        except:
            return 0.0
    
    def get_lidar_min_range(self):
        """Obtener rango mínimo del lidar"""
        try:
            return self.lidar_sensor.getMinRange()
        except:
            return 0.0
    
    def get_lidar_closest_obstacle(self):
        """Obtener la distancia al obstáculo más cercano detectado por el lidar"""
        try:
            ranges = self.get_lidar_data()
            if ranges and len(ranges) > 0:
                # Filtrar valores infinitos o inválidos
                valid_ranges = [r for r in ranges if r != float('inf') and r > 0]
                if valid_ranges:
                    return min(valid_ranges)
        except:
            pass
        return float('inf')
    
    def get_lidar_sectored_distances(self, num_sectors: int = 8, 
                                   min_range: float = 0.05, 
                                   max_range: float = 10.0) -> list:
        """
        Divide los datos del LiDAR en sectores direccionales y obtiene la distancia mínima de cada sector.
        
        Args:
            num_sectors (int): Número de sectores en que dividir el LiDAR (8 por defecto)
            min_range (float): Distancia mínima válida en metros
            max_range (float): Distancia máxima válida en metros (valor por defecto para obstáculos lejanos)
            
        Returns:
            list: Lista de distancias mínimas por sector [front, front_right, right, back_right, 
                  back, back_left, left, front_left] u orden equivalente según num_sectors
        """
        try:
            lidar_data = self.get_lidar_data()
            
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
    
    def get_lidar_fov(self):
        """Obtener campo de visión del lidar"""
        try:
            return self.lidar_sensor.getFov()
        except:
            return 0.0
    
    def get_obstacle_sensors(self, num_sectors=8):
        """
        Obtener sensores de obstáculos simples desde el LiDAR.
        Convierte datos del LiDAR en sectores para detección de obstáculos.
        
        Args:
            num_sectors (int): Número de sectores a crear (por defecto 8)
            
        Returns:
            List[float]: Lista con distancias mínimas por sector
        """
        lidar_data = self.get_lidar_data()
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
    
    def print_lidar_point_cloud(self, max_points=10):
        """Imprimir los valores de la nube de puntos del lidar
        
        Args:
            max_points (int): Número máximo de puntos a mostrar
        """
        try:
            point_cloud = self.get_lidar_point_cloud()
            ranges = self.get_lidar_data()
            
            print(f"--- LIDAR POINT CLOUD ---")
            print(f"Número total de puntos: {len(point_cloud) if point_cloud else 0}")
            print(f"Número de rangos: {len(ranges) if ranges else 0}")
            
            if point_cloud and len(point_cloud) > 0:
                print(f"Primeros {min(max_points, len(point_cloud))} puntos:")
                for i in range(min(max_points, len(point_cloud))):
                    if i < len(point_cloud):
                        point = point_cloud[i]
                        if len(point) >= 3:  # Verificar que tenga coordenadas x, y, z
                            print(f"  Punto {i}: x={point[0]:.3f}, y={point[1]:.3f}, z={point[2]:.3f}")
                        else:
                            print(f"  Punto {i}: {point}")
            
            if ranges and len(ranges) > 0:
                print(f"Primeros {min(max_points, len(ranges))} rangos:")
                for i in range(min(max_points, len(ranges))):
                    if i < len(ranges):
                        range_val = ranges[i]
                        if range_val != float('inf'):
                            print(f"  Rango {i}: {range_val:.3f}m")
                        else:
                            print(f"  Rango {i}: inf")
            
            print("------------------------")
            
        except Exception as e:
            print(f"Error al obtener nube de puntos: {e}")
    
    def print_lidar_summary(self):
        """Imprimir un resumen de la información del lidar"""
        try:
            ranges = self.get_lidar_data()
            point_cloud = self.get_lidar_point_cloud()
            
            print(f"--- LIDAR SUMMARY ---")
            print(f"Puntos totales: {self.get_lidar_range_count()}")
            print(f"Rango: {self.get_lidar_min_range():.2f} - {self.get_lidar_max_range():.2f}m")
            print(f"Campo de visión: {math.degrees(self.get_lidar_fov()):.1f}°")
            print(f"Obstáculo más cercano: {self.get_lidar_closest_obstacle():.3f}m")
            
            if ranges:
                valid_ranges = [r for r in ranges if r != float('inf') and r > 0]
                if valid_ranges:
                    print(f"Rangos válidos: {len(valid_ranges)}/{len(ranges)}")
                    print(f"Distancia promedio: {sum(valid_ranges)/len(valid_ranges):.3f}m")
                else:
                    print("No hay rangos válidos")
            
            print("--------------------")
            
        except Exception as e:
            print(f"Error en resumen del lidar: {e}")
    
    def cleanup(self):
        """Limpiar recursos al finalizar"""
        # Llamar al método de la interfaz base que detiene el robot
        super().cleanup()
    
    # Implementación de la interfaz IRobot
    def set_motor_speeds(self, left_speed: float, right_speed: float) -> None:
        """
        Establece las velocidades de los motores.
        
        Args:
            left_speed: Velocidad del motor izquierdo (rad/s)
            right_speed: Velocidad del motor derecho (rad/s)
        """
        # Debe ser implementado por subclases específicas
        raise NotImplementedError("Subclases deben implementar set_motor_speeds")
    
    def get_pose(self) -> RobotPose:
        """
        Obtiene la pose actual del robot.
        
        Returns:
            Pose actual del robot
        """
        gps_position = self.get_gps_position()
        compass_direction, angle = self.get_compass_orientation()
        
        return RobotPose(gps_position[0], gps_position[1], angle)
    
    def set_pose(self, x: float, y: float, theta: float) -> None:
        """
        Establece la pose del robot.
        
        Nota: En Webots esto normalmente no es posible durante la simulación.
        Este método existe para compatibilidad con la interfaz.
        """
        print(f"Advertencia: set_pose no está soportado en robots de Webots durante la simulación")
    
    def get_motor_speeds(self) -> Tuple[float, float]:
        """
        Obtiene las velocidades actuales de los motores.
        
        Returns:
            Tupla (velocidad_izquierda, velocidad_derecha) en rad/s
        """
        # Debe ser implementado por subclases específicas
        raise NotImplementedError("Subclases deben implementar get_motor_speeds")
    
    # Métodos abstractos que deben ser implementados por subclases
    def _init_motors(self):
        """Inicializar motores específicos del robot (debe ser implementado por subclases)"""
        raise NotImplementedError("Subclases deben implementar _init_motors")
    
    def move_forward(self, speed=2.0):
        """Mover el robot hacia adelante (debe ser implementado por subclases)"""
        raise NotImplementedError("Subclases deben implementar move_forward")
    
    def move_backward(self, speed=2.0):
        """Mover el robot hacia atrás (debe ser implementado por subclases)"""
        raise NotImplementedError("Subclases deben implementar move_backward")
    
    def turn_left(self, speed=2.0):
        """Girar el robot a la izquierda (debe ser implementado por subclases)"""
        raise NotImplementedError("Subclases deben implementar turn_left")
    
    def turn_right(self, speed=2.0):
        """Girar el robot a la derecha (debe ser implementado por subclases)"""
        raise NotImplementedError("Subclases deben implementar turn_right")
