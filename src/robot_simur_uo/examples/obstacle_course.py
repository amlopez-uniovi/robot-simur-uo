"""
Ejemplo de navegación en un curso con obstáculos.
"""

import random
import math

from robot_simur_uo.controllers.obstacle_avoidance import ObstacleAvoidanceController
from robot_simur_uo.controllers.navigation import NavigationController
from robot_simur_uo.sensors.distance_sensors import DistanceSensorProcessor
from robot_simur_uo.utils.coordinates import RobotPose
from robot_simur_uo.utils.visualization import create_simple_map
from robot_simur_uo.utils.simulated_differential_robot import SimulatedDifferentialRobot


class ObstacleCourseExample:
    """Ejemplo de navegación evitando obstáculos."""
    
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
            
        self.navigator = NavigationController(max_speed=0.3)
        self.obstacle_avoider = ObstacleAvoidanceController(safe_distance=0.4)
        self.sensor_processor = DistanceSensorProcessor(num_sensors=8)
        
        self.current_pose = RobotPose(0.0, 0.0, 0.0)
        self.goal = (3.0, 3.0)
        
        # Crear obstáculos simulados
        self.obstacles = [
            (1.0, 0.5), (1.5, 1.5), (0.5, 2.0),
            (2.0, 1.0), (2.5, 2.5), (1.0, 3.0)
        ]
        
        self.path_history = []  # Para registrar el camino recorrido
        
    def run_example(self, time_step: float = 0.032, max_iterations: int = 2000):
        """
        Ejecuta el ejemplo de navegación con obstáculos.
        
        Args:
            time_step: Paso de tiempo de simulación
            max_iterations: Máximo número de iteraciones
        """
        print("=== Ejemplo de Curso con Obstáculos ===")
        print(f"Robot: {type(self.robot).__name__}")
        print(f"Objetivo: {self.goal}")
        print(f"Obstáculos: {len(self.obstacles)} obstáculos")
        print()
        
        self.navigator.set_target(self.goal[0], self.goal[1])
        
        for iteration in range(max_iterations):
            current_x, current_y, current_theta = self.current_pose.to_tuple()
            
            # Registrar posición en historial
            if iteration % 10 == 0:  # Cada 10 iteraciones
                self.path_history.append((current_x, current_y))
            
            # Simular lecturas de sensores
            sensor_readings = self._simulate_distance_sensors()
            
            # Verificar si hay obstáculos
            if self.obstacle_avoider.is_path_clear(sensor_readings):
                # Camino despejado, usar navegación normal
                left_speed, right_speed = self.navigator.calculate_motor_speeds(
                    current_x, current_y, current_theta
                )
            else:
                # Hay obstáculos, usar evitación
                sensor_angles = self.sensor_processor.sensor_angles
                left_speed, right_speed = self.obstacle_avoider.calculate_avoidance_speeds(
                    sensor_readings, sensor_angles, base_speed=0.2
                )
            
            # Simular movimiento
            self._simulate_robot_movement(left_speed, right_speed, time_step)
            
            # Verificar si llegamos al objetivo
            if self.navigator.is_target_reached(current_x, current_y, tolerance=0.3):
                print(f"🎉 ¡Objetivo alcanzado en {iteration + 1} iteraciones!")
                break
            
            # Mostrar progreso
            if iteration % 100 == 0:
                self._show_progress(iteration, sensor_readings)
            
            # Visualización periódica
            if iteration % 200 == 0:
                self._show_current_situation()
        
        self._show_final_results(iteration + 1)
    
    def _simulate_distance_sensors(self) -> list:
        """
        Simula las lecturas de sensores de distancia.
        
        Returns:
            Lista de distancias a obstáculos
        """
        sensor_readings = []
        current_x, current_y, current_theta = self.current_pose.to_tuple()
        
        for sensor_angle in self.sensor_processor.sensor_angles:
            # Ángulo absoluto del sensor
            absolute_angle = current_theta + sensor_angle
            
            # Simular rayo del sensor
            max_range = 1.0
            min_distance = max_range
            
            # Verificar intersección con obstáculos
            for obs_x, obs_y in self.obstacles:
                # Distancia al obstáculo
                obs_distance = math.sqrt((obs_x - current_x)**2 + (obs_y - current_y)**2)
                
                # Ángulo hacia el obstáculo
                obs_angle = math.atan2(obs_y - current_y, obs_x - current_x)
                
                # Diferencia angular
                angle_diff = abs(absolute_angle - obs_angle)
                angle_diff = min(angle_diff, 2*math.pi - angle_diff)
                
                # Si el obstáculo está en el cono del sensor
                if angle_diff < 0.2 and obs_distance < min_distance:  # Cono de 0.2 radianes
                    min_distance = obs_distance - 0.1  # Radio del obstáculo
            
            # Añadir ruido
            noise = random.uniform(-0.05, 0.05)
            min_distance = max(0.0, min_distance + noise)
            
            sensor_readings.append(min_distance)
        
        return sensor_readings
    
    def _simulate_robot_movement(self, left_speed: float, right_speed: float, dt: float):
        """Simula el movimiento del robot."""
        wheel_radius = 0.025
        wheel_base = 0.053
        
        v_left = left_speed * wheel_radius
        v_right = right_speed * wheel_radius
        
        v_linear = (v_left + v_right) / 2
        v_angular = (v_right - v_left) / wheel_base
        
        current_theta = self.current_pose.theta
        dx = v_linear * math.cos(current_theta) * dt
        dy = v_linear * math.sin(current_theta) * dt
        dtheta = v_angular * dt
        
        self.current_pose.update(dx, dy, dtheta)
    
    def _show_progress(self, iteration: int, sensor_readings: list):
        """Muestra el progreso actual."""
        current_x, current_y, _ = self.current_pose.to_tuple()
        distance_to_goal = math.sqrt((self.goal[0] - current_x)**2 + (self.goal[1] - current_y)**2)
        
        min_sensor = min(sensor_readings) if sensor_readings else float('inf')
        
        print(f"Iter {iteration:4d}: Pos({current_x:.2f}, {current_y:.2f}), "
              f"Dist objetivo: {distance_to_goal:.2f}m, "
              f"Sensor min: {min_sensor:.2f}m")
    
    def _show_current_situation(self):
        """Muestra la situación actual en el mapa."""
        print("\n--- Situación Actual ---")
        current_pos = (self.current_pose.x, self.current_pose.y)
        
        map_vis = create_simple_map(
            robot_pos=current_pos,
            obstacles=self.obstacles,
            goal=self.goal,
            bounds=(-1, 4, -1, 4)
        )
        
        print(map_vis)
        print()
    
    def _show_final_results(self, iterations: int):
        """Muestra los resultados finales."""
        print(f"\n=== Resultados Finales ===")
        print(f"Iteraciones totales: {iterations}")
        
        # Calcular distancia total recorrida
        total_distance = 0.0
        for i in range(1, len(self.path_history)):
            prev_x, prev_y = self.path_history[i-1]
            curr_x, curr_y = self.path_history[i]
            total_distance += math.sqrt((curr_x - prev_x)**2 + (curr_y - prev_y)**2)
        
        print(f"Distancia recorrida: {total_distance:.2f}m")
        
        # Distancia directa al objetivo
        direct_distance = math.sqrt(self.goal[0]**2 + self.goal[1]**2)
        efficiency = direct_distance / total_distance if total_distance > 0 else 0
        print(f"Eficiencia del camino: {efficiency:.2%}")
        
        # Visualización final
        print("\n--- Mapa Final ---")
        final_pos = (self.current_pose.x, self.current_pose.y)
        
        map_vis = create_simple_map(
            robot_pos=final_pos,
            obstacles=self.obstacles,
            goal=self.goal,
            bounds=(-1, 4, -1, 4)
        )
        
        print(map_vis)


def run_obstacle_course_demo():
    """Función conveniente para ejecutar el demo."""
    print("Iniciando demo de curso con obstáculos...")
    
    example = ObstacleCourseExample('epuck')
    example.run_example()
    
    print("\n" + "="*50)
    input("Presiona Enter para continuar...")


if __name__ == "__main__":
    run_obstacle_course_demo()
