"""
Ejemplo usando NavigationController con comandos Ackermann.
Demuestra cómo usar el controlador de navegación refactorizado.
"""

import math
from ..controllers.navigation import NavigationController
from ..utils.visualization import DataVisualizer
from ..interfaces import IRobotBase
from .basic_navigation import SimulatedDifferentialRobot, SimulatedAckermannRobot


class NavigationControllerExample:
    """Ejemplo usando NavigationController refactorizado."""
    
    def __init__(self, robot: IRobotBase):
        """
        Inicializa el ejemplo.
        
        Args:
            robot: Instancia del robot que implementa IRobotBase
        """
        self.robot = robot
        self.controller = NavigationController(max_speed=0.5, linear_gain=1.5, angular_gain=1.0)
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
        Ejecuta el ejemplo de navegación usando NavigationController.
        
        Args:
            time_step: Paso de tiempo de simulación
            max_iterations: Máximo número de iteraciones
        """
        print("=== Ejemplo con NavigationController ===")
        print(f"Robot: {type(self.robot).__name__}")
        print(f"Waypoints: {self.waypoints}")
        print()
        
        for iteration in range(max_iterations):
            # Obtener waypoint actual
            if self.current_waypoint_index < len(self.waypoints):
                target = self.waypoints[self.current_waypoint_index]
                self.controller.set_target(target[0], target[1], tol=0.05)
                
                # Obtener pose actual del robot
                current_pose = self.robot.get_pose()
                current_x, current_y, current_theta = current_pose.to_tuple()
                
                # Usar NavigationController para calcular comandos Ackermann
                drive_speed, steering_angle = self.controller.calculate_control_commands(
                    current_x, current_y, current_theta
                )
                
                # Aplicar comandos usando interfaz unificada
                self.robot.set_drive_speed(drive_speed)
                self.robot.set_steering_angle(steering_angle)
                self.robot.step(time_step)
                
                # Verificar si llegamos al objetivo
                if self.controller.is_target_reached(current_x, current_y):
                    print(f"✓ Waypoint {self.current_waypoint_index + 1} alcanzado: {target}")
                    self.current_waypoint_index += 1
                    
                    if self.current_waypoint_index >= len(self.waypoints):
                        print("🎉 ¡Todos los waypoints completados!")
                        break
                
                # Mostrar progreso cada 50 iteraciones
                if iteration % 50 == 0:
                    distance = math.sqrt((target[0] - current_x)**2 + (target[1] - current_y)**2)
                    print(f"Posición: ({current_x:.2f}, {current_y:.2f}), "
                          f"Objetivo: {target}, Distancia: {distance:.2f}m")
            
            else:
                break
        
        print(f"\nEjemplo completado en {iteration + 1} iteraciones")
        self._show_final_visualization()
    
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
        
        # Dibujar trayectoria
        self.visualizer.draw_path(self.waypoints)
        
        self.visualizer.print_visualization()


def run_navigation_controller_demo():
    """Función conveniente para ejecutar el demo con NavigationController."""
    print("=== Demo NavigationController Refactorizado ===")
    
    # Ejemplo con robot diferencial
    print("\n1. Robot Diferencial:")
    diff_robot = SimulatedDifferentialRobot()
    diff_example = NavigationControllerExample(diff_robot)
    diff_example.run_example(max_iterations=500)
    
    # Ejemplo con robot Ackermann
    print("\n2. Robot Ackermann:")
    ack_robot = SimulatedAckermannRobot()
    ack_example = NavigationControllerExample(ack_robot)
    ack_example.run_example(max_iterations=500)


if __name__ == "__main__":
    run_navigation_controller_demo()
