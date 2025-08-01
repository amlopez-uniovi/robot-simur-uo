"""
Ejemplo básico de navegación para un robot.
"""

from ..controllers.navigation import NavigationController
from ..utils.coordinates import RobotPose
from .simulated_differential_robot import SimulatedDifferentialRobot
from .simulated_ackermann_robot import SimulatedAckermannRobot
from ..utils.visualization import DataVisualizer
from ..interfaces import IRobotBase


class BasicNavigationExample:
    """Ejemplo de navegación básica punto a punto."""
    
    def __init__(self, robot: IRobotBase):
        """
        Inicializa el ejemplo.
        
        Args:
            robot: Instancia del robot que implementa IRobotBase (interfaz Ackermann unificada)
        """
        self.robot = robot
        self.navigator = NavigationController(max_speed=0.5)
        self.visualizer = DataVisualizer(80, 24)
        
        # Lista de objetivos a visitar (waypoints simples)
        self.waypoints = [
            (1.0, 0.0),
            (0.0, 0.0)
        ]
        self.current_waypoint_index = 0
        
    def _control_robot(self, left_speed: float, right_speed: float):
        """
        Controla el robot usando la interfaz Ackermann unificada.
        Convierte velocidades diferenciales a velocidad lineal + ángulo de dirección.
        
        Args:
            left_speed: Velocidad calculada para motor izquierdo (robots diferenciales)
            right_speed: Velocidad calculada para motor derecho (robots diferenciales)
        """
        # Convertir velocidades diferenciales a modelo Ackermann
        linear_speed = (left_speed + right_speed) / 2.0
        speed_diff = right_speed - left_speed
        steering_angle = speed_diff * 0.5  # Factor de escala ajustable
        
        # Limitar el ángulo de dirección (±30 grados = ±0.52 rad)
        max_steering = 0.52
        steering_angle = max(-max_steering, min(max_steering, steering_angle))
        
        # Usar interfaz Ackermann unificada
        self.robot.set_drive_speed(linear_speed)
        self.robot.set_steering_angle(steering_angle)
    
        
    def run_example(self, time_step: float = 0.032, max_iterations: int = 1000):
        """
        Ejecuta el ejemplo de navegación.
        
        Args:
            time_step: Paso de tiempo de simulación
            max_iterations: Máximo número de iteraciones
        """
        print("=== Ejemplo de Navegación Básica ===")
        print(f"Robot: {type(self.robot).__name__}")
        print(f"Waypoints: {self.waypoints}")
        print()
        
        for iteration in range(max_iterations):
            # Obtener waypoint actual
            if self.current_waypoint_index < len(self.waypoints):
                target = self.waypoints[self.current_waypoint_index]
                self.navigator.set_target(target[0], target[1])
                
                # Obtener pose actual del robot
                current_pose = self.robot.get_pose()
                current_x, current_y, current_theta = current_pose.to_tuple()
                
                # Calcular velocidades de motores
                left_speed, right_speed = self.navigator.calculate_motor_speeds(
                    current_x, current_y, current_theta
                )
                
                # Establecer velocidades en el robot de forma polimórfica
                self._control_robot(left_speed, right_speed)
                self.robot.step(time_step)
                
                # Verificar si llegamos al objetivo
                if self.navigator.is_target_reached(current_x, current_y):
                    print(f"✓ Waypoint {self.current_waypoint_index + 1} alcanzado: {target}")
                    self.current_waypoint_index += 1
                    
                    if self.current_waypoint_index >= len(self.waypoints):
                        print("🎉 ¡Todos los waypoints completados!")
                        break
                
                # Mostrar progreso cada 50 iteraciones
                if iteration % 50 == 0:
                    self._show_progress()
            
            else:
                break
        
        print(f"\nEjemplo completado en {iteration + 1} iteraciones")
        self._show_final_visualization()
    
    def _show_progress(self):
        """Muestra el progreso actual."""
        current_pose = self.robot.get_pose()
        current_x, current_y, current_theta = current_pose.to_tuple()
        
        if self.current_waypoint_index < len(self.waypoints):
            target = self.waypoints[self.current_waypoint_index]
            distance = ((target[0] - current_x)**2 + (target[1] - current_y)**2)**0.5
            
            print(f"Posición: ({current_x:.2f}, {current_y:.2f}), "
                  f"Objetivo: {target}, Distancia: {distance:.2f}m")
    
    def _show_final_visualization(self):
        """Muestra visualización final del recorrido."""
        print("\n=== Visualización Final ===")
        
        # Configurar visualizador
        self.visualizer.set_bounds(-0.5, 1.5, -0.5, 1.5)
        self.visualizer.clear()
        
        # Dibujar waypoints
        for i, (x, y) in enumerate(self.waypoints):
            self.visualizer.draw_point(x, y, str(i+1))
        
        # Dibujar posición final del robot
        final_pose = self.robot.get_pose()
        final_x, final_y, final_theta = final_pose.to_tuple()
        self.visualizer.draw_robot(final_x, final_y, final_theta)
        
        # Dibujar trayectoria (simplificada)
        self.visualizer.draw_path(self.waypoints)
        
        self.visualizer.print_visualization()


def run_basic_navigation_demo():
    """Función conveniente para ejecutar el demo."""
    print("=== Demo de Navegación Básica ===")
    
    # Ejemplo con robot diferencial
    print("\n1. Robot Diferencial:")
    diff_robot = SimulatedDifferentialRobot()
    diff_example = BasicNavigationExample(diff_robot)
    diff_example.waypoints = [(0.5, 0.0), (0.0, 0.0)]  # Waypoints simples
    diff_example.current_waypoint_index = 0
    diff_example.run_example(max_iterations=200)
    
    # Ejemplo con robot Ackermann
    print("\n2. Robot Ackermann:")
    ack_robot = SimulatedAckermannRobot()
    ack_example = BasicNavigationExample(ack_robot)
    ack_example.waypoints = [(0.5, 0.0), (0.0, 0.0)]  # Waypoints simples
    ack_example.current_waypoint_index = 0
    ack_example.run_example(max_iterations=200)


if __name__ == "__main__":
    run_basic_navigation_demo()
