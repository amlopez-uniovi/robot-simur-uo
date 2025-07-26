"""
Ejemplo básico de navegación con robot.
"""

from robot_simur_uo.controllers import NavigationController
from robot_simur_uo.utils import DataVisualizer, SimulatedRobot
from robot_simur_uo.interfaces import IRobot


class BasicNavigationExample:
    """Ejemplo de navegación básica punto a punto."""
    
    def __init__(self, robot: IRobot):
        """
        Inicializa el ejemplo.
        
        Args:
            robot: Instancia del robot que implementa IRobot
        """
        self.robot = robot
        self.navigator = NavigationController(max_speed=0.5)
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
                
                # Obtener pose actual del robot
                current_pose = self.robot.get_pose()
                current_x, current_y, current_theta = current_pose.to_tuple()
                
                # Calcular velocidades de motores
                left_speed, right_speed = self.navigator.calculate_motor_speeds(
                    current_x, current_y, current_theta
                )
                
                # Establecer velocidades en el robot y simular movimiento
                self.robot.set_motor_speeds(left_speed, right_speed)
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
    print("Iniciando demo de navegación básica...")
    
    # Crear robot simulado
    robot = SimulatedRobot()
    
    # Crear y ejecutar ejemplo con el robot
    example = BasicNavigationExample(robot)
    example.run_example()
    
    print("\n" + "="*50)
    input("Presiona Enter para continuar...")


def run_demo_with_robot_type(robot_type: str = 'simulated'):
    """
    Ejecuta el demo con un tipo específico de robot.
    
    Args:
        robot_type: Tipo de robot ('simulated', 'epuck', 'rosbot')
    """
    print(f"Iniciando demo de navegación básica con robot {robot_type}...")
    
    # Crear robot según el tipo especificado
    if robot_type.lower() == 'simulated':
        robot = SimulatedRobot()
    elif robot_type.lower() == 'epuck':
        from robot_simur_uo.webots import EPuck
        robot = EPuck()
    elif robot_type.lower() == 'rosbot':
        from robot_simur_uo.webots import RosBot
        robot = RosBot()
    else:
        raise ValueError(f"Tipo de robot no soportado: {robot_type}. Use 'simulated', 'epuck' o 'rosbot'")
    
    # Crear y ejecutar ejemplo con el robot
    example = BasicNavigationExample(robot)
    example.run_example()
    
    print("\n" + "="*50)
    input("Presiona Enter para continuar...")


if __name__ == "__main__":
    import sys
    
    # Permitir especificar el tipo de robot como argumento
    if len(sys.argv) > 1:
        robot_type = sys.argv[1]
        run_demo_with_robot_type(robot_type)
    else:
        # Ejecutar demo por defecto con robot simulado
        run_basic_navigation_demo()
        
        # Ejemplo adicional mostrando flexibilidad
        print("\n" + "="*50)
        print("Ejemplo adicional: Creación manual del robot")
        print("="*50)
        
        # Crear robot con parámetros personalizados
        custom_robot = SimulatedRobot(wheel_radius=0.03, wheel_base=0.06)
        
        # Usar el robot con el ejemplo
        custom_example = BasicNavigationExample(custom_robot)
        
        # Configurar waypoints diferentes
        custom_example.waypoints = [(0.5, 0.5), (0.0, 0.0)]
        custom_example.current_waypoint_index = 0
        
        print("Ejecutando con robot personalizado y waypoints diferentes...")
        custom_example.run_example(max_iterations=500)
