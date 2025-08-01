"""
Demonstración de uso de sensores.
"""

import time
import numpy as np
import random
import math

from robot_simur_uo.sensors.distance_sensors import DistanceSensorProcessor
from robot_simur_uo.sensors.lidar_processor import LidarProcessor
from robot_simur_uo.sensors.camera_processor import CameraProcessor
from robot_simur_uo.sensors.sensor_fusion import SensorFusion
from robot_simur_uo.utils.visualization import DataVisualizer
from robot_simur_uo.utils.simulated_differential_robot import SimulatedDifferentialRobot


class SensorDemoExample:
    """Ejemplo de demostración de capacidades de sensores."""
    
    def __init__(self, robot_type: str = 'simulated'):
        """
        Inicializa el ejemplo.
        
        Args:
            robot_type: Tipo de robot a usar ('simulated', 'epuck', 'rosbot')
        """
        # Crear robot explícitamente según el tipo
        if robot_type.lower() == 'simulated':
            self.robot = SimulatedDifferentialRobot()
        elif robot_type.lower() == 'epuck':
            from robot_simur_uo.webots import EPuck
            self.robot = EPuck()
        elif robot_type.lower() == 'rosbot':
            from robot_simur_uo.webots import RosBot
            self.robot = RosBot()
        else:
            raise ValueError(f"Tipo de robot no soportado: {robot_type}. Use 'simulated', 'epuck' o 'rosbot'")
        
        # Inicializar procesadores de sensores
        self.distance_processor = DistanceSensorProcessor(num_sensors=8)
        self.lidar_processor = LidarProcessor(max_range=5.0)
        self.camera_processor = CameraProcessor(width=320, height=240)
        self.sensor_fusion = SensorFusion()
        
        self.visualizer = DataVisualizer(80, 30)
        
        # Simular entorno con obstáculos
        self.obstacles = [
            (2.0, 0.0), (1.5, 1.0), (-1.0, 0.5),
            (0.0, 2.0), (-2.0, -1.0), (3.0, 1.5)
        ]
        
    def run_demo(self, num_iterations: int = 50):
        """
        Ejecuta la demostración de sensores.
        
        Args:
            num_iterations: Número de iteraciones de la demo
        """
        print("=== Demostración de Sensores ===")
        print(f"Robot: {type(self.robot).__name__}")
        print(f"Sensores disponibles: Distancia, LiDAR, Cámara")
        print()
        
        for iteration in range(num_iterations):
            print(f"\n--- Iteración {iteration + 1} ---")
            
            # Simular datos de sensores
            distance_data = self._simulate_distance_sensors()
            lidar_data = self._simulate_lidar_scan()
            camera_data = self._simulate_camera_image()
            
            # Procesar datos individualmente
            self._process_distance_sensors(distance_data)
            self._process_lidar_data(lidar_data)
            self._process_camera_data(camera_data)
            
            # Fusionar datos de sensores
            self._demonstrate_sensor_fusion(distance_data, lidar_data)
            
            # Visualización cada 10 iteraciones
            if iteration % 10 == 0:
                self._show_sensor_visualization(distance_data, lidar_data)
            
            # Pausa entre iteraciones (en una aplicación real esto sería el tiempo de simulación)
            if iteration < 5:  # Mostrar las primeras iteraciones en detalle
                input("Presiona Enter para continuar...")
        
        self._show_final_summary()
    
    def _simulate_distance_sensors(self) -> list:
        """Simula datos de sensores de distancia ultrasónicos."""
        readings = []
        
        for i in range(8):
            # Simular medición con algo de variabilidad
            base_distance = 1.0 + random.uniform(-0.3, 0.3)
            noise = random.uniform(-0.05, 0.05)
            distance = max(0.1, base_distance + noise)
            readings.append(distance)
        
        return readings
    
    def _simulate_lidar_scan(self) -> dict:
        """Simula un escaneo LiDAR completo."""
        num_points = 360  # 1 punto por grado
        ranges = []
        angles = []
        
        for i in range(num_points):
            angle = math.radians(i - 180)  # -180 a +179 grados
            angles.append(angle)
            
            # Simular distancia basada en obstáculos
            distance = 5.0  # Rango máximo
            
            # Verificar obstáculos en esta dirección
            for obs_x, obs_y in self.obstacles:
                obs_angle = math.atan2(obs_y, obs_x)
                angle_diff = abs(angle - obs_angle)
                
                if angle_diff < 0.1:  # Dentro del haz
                    obs_distance = math.sqrt(obs_x**2 + obs_y**2)
                    distance = min(distance, obs_distance + random.uniform(-0.1, 0.1))
            
            ranges.append(distance)
        
        return {'ranges': ranges, 'angles': angles}
    
    def _simulate_camera_image(self) -> list:
        """Simula datos de imagen de cámara (simplificado)."""
        # Generar imagen simulada (escala de grises)
        width, height = 320, 240
        image_data = []
        
        for y in range(height):
            for x in range(width):
                # Simular patrón de imagen con algo de estructura
                value = int(128 + 50 * math.sin(x/20) * math.cos(y/20))
                value += random.randint(-20, 20)  # Ruido
                value = max(0, min(255, value))
                image_data.append(value)
        
        return image_data
    
    def _process_distance_sensors(self, data: list):
        """Procesa datos de sensores de distancia."""
        print("📏 Sensores de Distancia:")
        
        # Filtrar datos
        filtered_data = self.distance_processor.filter_readings(data, "median")
        
        # Detectar obstáculos por sectores
        sectors = self.distance_processor.detect_obstacles_by_sector(
            filtered_data, obstacle_threshold=0.8
        )
        
        # Mostrar resultados
        print(f"   Datos brutos:    {[f'{x:.2f}' for x in data[:4]]}... (8 sensores)")
        print(f"   Datos filtrados: {[f'{x:.2f}' for x in filtered_data[:4]]}...")
        
        print("   Obstáculos por sector:")
        for sector, distance in sectors.items():
            if distance < float('inf'):
                print(f"     {sector:12}: {distance:.2f}m")
        
        # Verificar seguridad
        is_safe = self.distance_processor.is_safe_to_move(filtered_data)
        print(f"   ¿Seguro avanzar?: {'✓ Sí' if is_safe else '✗ No'}")
    
    def _process_lidar_data(self, data: dict):
        """Procesa datos del LiDAR."""
        print("\n🔍 LiDAR:")
        
        ranges = data['ranges']
        angles = data['angles']
        
        # Filtrar ruido
        filtered_ranges = self.lidar_processor.filter_noise(ranges)
        
        # Detectar obstáculos
        obstacles = self.lidar_processor.detect_obstacles(
            filtered_ranges, angles, min_distance=3.0
        )
        
        # Encontrar espacios libres
        gaps = self.lidar_processor.find_gaps(
            filtered_ranges, angles, gap_threshold=3.0
        )
        
        # Obtener distancia frontal
        front_clearance = self.lidar_processor.calculate_front_clearance(
            filtered_ranges, angles
        )
        
        print(f"   Puntos de escaneo: {len(ranges)}")
        print(f"   Obstáculos detectados: {len(obstacles)}")
        print(f"   Espacios libres: {len(gaps)}")
        print(f"   Distancia frontal: {front_clearance:.2f}m")
        
        if obstacles:
            closest = min(obstacles, key=lambda x: x[1])
            angle_deg = math.degrees(closest[0])
            print(f"   Obstáculo más cercano: {closest[1]:.2f}m a {angle_deg:.0f}°")
    
    def _process_camera_data(self, data: list):
        """Procesa datos de la cámara."""
        print("\n📷 Cámara:")
        
        # Detectar centro de línea (para seguimiento de líneas)
        line_center = self.camera_processor.detect_line_center(data, threshold=100)
        
        # Detectar objetos por color
        bright_objects = self.camera_processor.detect_objects_by_color(
            data, (200, 255)  # Objetos brillantes
        )
        
        # Calcular brillo promedio
        brightness = self.camera_processor.get_image_brightness(data)
        
        print(f"   Resolución: {self.camera_processor.width}x{self.camera_processor.height}")
        print(f"   Brillo promedio: {brightness:.1f}")
        
        if line_center is not None:
            print(f"   Centro de línea: {line_center:.2f} (normalizado)")
        else:
            print("   Centro de línea: No detectado")
        
        print(f"   Objetos brillantes: {len(bright_objects)}")
        
        for i, (x, y, size) in enumerate(bright_objects[:3]):  # Mostrar solo los primeros 3
            distance = self.camera_processor.calculate_object_distance(size)
            print(f"     Objeto {i+1}: Pos({x}, {y}), Tamaño: {size}, Dist: {distance:.2f}m")
    
    def _demonstrate_sensor_fusion(self, distance_data: list, lidar_data: dict):
        """Demuestra la fusión de datos de sensores."""
        print("\n🔀 Fusión de Sensores:")
        
        # Simular mediciones de distancia de diferentes sensores
        measurements = {
            'ultrasonic': min(distance_data),
            'lidar': min(lidar_data['ranges']),
            'ir': min(distance_data) + random.uniform(-0.1, 0.1)
        }
        
        # Fusionar mediciones
        fused_distance = self.sensor_fusion.fuse_distance_measurements(measurements)
        
        print(f"   Mediciones individuales:")
        for sensor, distance in measurements.items():
            confidence = self.sensor_fusion.confidence_weights.get(sensor, 0.5)
            print(f"     {sensor:12}: {distance:.2f}m (confianza: {confidence:.1f})")
        
        print(f"   Distancia fusionada: {fused_distance:.2f}m")
        
        # Evaluar salud de sensores (simulado)
        for sensor_type in ['lidar', 'ultrasonic', 'camera']:
            # Simular datos de salud
            self.sensor_fusion.add_measurement(sensor_type, random.uniform(0.5, 2.0))
            health = self.sensor_fusion.get_sensor_health(sensor_type)
            print(f"   Estado {sensor_type}: {health['status']}")
    
    def _show_sensor_visualization(self, distance_data: list, lidar_data: dict):
        """Muestra visualización de datos de sensores."""
        print("\n📊 Visualización de Sensores:")
        
        # Crear gráfico de sensores de distancia
        sensor_chart = self.visualizer.create_sensor_chart({
            'US_Front': [distance_data[0]],
            'US_Left': [distance_data[2]],
            'US_Right': [distance_data[6]],
            'US_Back': [distance_data[4]]
        })
        
        print(sensor_chart)
    
    def _show_final_summary(self):
        """Muestra resumen final de la demostración."""
        print("\n" + "="*50)
        print("🎯 RESUMEN DE LA DEMOSTRACIÓN")
        print("="*50)
        
        print("\n✅ Capacidades demostradas:")
        print("   • Procesamiento de sensores de distancia")
        print("   • Análisis de datos LiDAR")
        print("   • Procesamiento básico de imágenes")
        print("   • Fusión de datos multi-sensor")
        print("   • Detección de obstáculos")
        print("   • Evaluación de seguridad")
        
        print("\n🔧 Algoritmos utilizados:")
        print("   • Filtrado de ruido (mediana)")
        print("   • Detección por sectores")
        print("   • Búsqueda de espacios libres")
        print("   • Fusión ponderada por confianza")
        print("   • Flood fill para detección de objetos")
        
        print("\n💡 Aplicaciones prácticas:")
        print("   • Navegación autónoma")
        print("   • Evitación de obstáculos")
        print("   • Seguimiento de líneas")
        print("   • Mapeo y localización")
        print("   • Inspección automatizada")


def run_sensor_demo():
    """Función conveniente para ejecutar el demo."""
    print("Iniciando demostración de sensores...")
    
    example = SensorDemoExample('rosbot')
    example.run_demo(num_iterations=10)
    
    print("\n" + "="*50)
    input("Presiona Enter para continuar...")


if __name__ == "__main__":
    run_sensor_demo()
