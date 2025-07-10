"""
Ejemplo básico de navegación con robot.
"""

from ..webots.robot_factory import create_robot
from ..controllers.navigation import NavigationController
from ..utils.coordinates import RobotPose
from ..utils.visualization import DataVisualizer


class BasicNavigationExample:
    """Ejemplo de navegación básica punto a punto."""
    
    def __init__(self, robot_type: str = 'epuck'):
        """
        Inicializa el ejemplo.
        
        Args:
            robot_type: Tipo de robot a usar
        """
        self.robot = create_robot(robot_type)
        self.navigator = NavigationController(max_speed=0.5)
        self.current_pose = RobotPose(0.0, 0.0, 0.0)
        self.visualizer = DataVisualizer(80, 24)
        
        # Lista de objetivos a visitar
        self.waypoints = [
            (1.0, 0.0),
            (1.0, 1.0),
            (0.0, 1.0),
            (0.0, 0.0)
        ]
        self.current_waypoint_index = 0
        
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
                
                # Simular obtención de pose actual (en Webots sería del robot)
                current_x, current_y, current_theta = self.current_pose.to_tuple()
                
                # Calcular velocidades de motores
                left_speed, right_speed = self.navigator.calculate_motor_speeds(
                    current_x, current_y, current_theta
                )
                
                # Simular movimiento del robot
                self._simulate_robot_movement(left_speed, right_speed, time_step)
                
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
    
    def _simulate_robot_movement(self, left_speed: float, right_speed: float, dt: float):
        """
        Simula el movimiento del robot basado en velocidades de motores.
        
        Args:
            left_speed: Velocidad motor izquierdo
            right_speed: Velocidad motor derecho
            dt: Paso de tiempo
        """
        # Parámetros del robot (simplificados)
        wheel_radius = 0.025  # metros
        wheel_base = 0.053    # metros
        
        # Calcular velocidades lineales de las ruedas
        v_left = left_speed * wheel_radius
        v_right = right_speed * wheel_radius
        
        # Calcular velocidad lineal y angular del robot
        v_linear = (v_left + v_right) / 2
        v_angular = (v_right - v_left) / wheel_base
        
        # Actualizar pose
        import math
        current_theta = self.current_pose.theta
        
        # Integración simple
        dx = v_linear * math.cos(current_theta) * dt
        dy = v_linear * math.sin(current_theta) * dt
        dtheta = v_angular * dt
        
        self.current_pose.update(dx, dy, dtheta)
    
    def _show_progress(self):
        """Muestra el progreso actual."""
        current_x, current_y, current_theta = self.current_pose.to_tuple()
        
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
        final_x, final_y, final_theta = self.current_pose.to_tuple()
        self.visualizer.draw_robot(final_x, final_y, final_theta)
        
        # Dibujar trayectoria (simplificada)
        self.visualizer.draw_path(self.waypoints)
        
        self.visualizer.print_visualization()


def run_basic_navigation_demo():
    """Función conveniente para ejecutar el demo."""
    print("Iniciando demo de navegación básica...")
    
    # Crear y ejecutar ejemplo con E-puck
    example = BasicNavigationExample('epuck')
    example.run_example()
    
    print("\n" + "="*50)
    input("Presiona Enter para continuar...")


if __name__ == "__main__":
    run_basic_navigation_demo()
