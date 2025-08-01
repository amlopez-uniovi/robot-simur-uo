"""
Ejemplo básico de navegación para un robot.
Incluye implementaciones simuladas de robots para demostración.
"""

import math
from ..controllers.navigation import NavigationController
from ..utils.coordinates import RobotPose
from ..utils.visualization import DataVisualizer
from ..interfaces import IRobotBase
from ..interfaces.idifferential_robot import IDifferentialRobot
from ..interfaces.iackermann_robot import IAckermannRobot


class SimulatedDifferentialRobot(IDifferentialRobot):
    """
    Implementación simulada de robot diferencial.
    
    Usa la funcionalidad común de IDifferentialRobot y solo implementa
    la cinemática específica de simulación.
    """
    
    def __init__(self, wheel_radius: float = 0.0205, wheel_base: float = 0.117):
        """
        Inicializa el robot diferencial simulado.
        
        Args:
            wheel_radius: Radio de la rueda (metros)
            wheel_base: Distancia entre ruedas (metros)
        """
        super().__init__(wheel_radius, wheel_base)
    
    def step(self, dt: float) -> None:
        """
        Implementación simulada del paso de tiempo.
        
        Usa el modelo cinemático diferencial para actualizar la pose.
        
        Args:
            dt: Paso de tiempo en segundos
        """
        # Velocidades lineales de las ruedas
        v_left = self.left_speed * self.wheel_radius
        v_right = self.right_speed * self.wheel_radius
        
        # Velocidad lineal y angular del robot
        v = (v_left + v_right) / 2.0
        omega = (v_right - v_left) / self.wheel_base
        
        if abs(omega) < 1e-6:
            # Movimiento recto
            dx = v * math.cos(self.pose.theta) * dt
            dy = v * math.sin(self.pose.theta) * dt
            dtheta = 0.0
        else:
            # Movimiento con giro
            R = v / omega
            dtheta = omega * dt
            dx = R * (math.sin(self.pose.theta + dtheta) - math.sin(self.pose.theta))
            dy = R * (-math.cos(self.pose.theta + dtheta) + math.cos(self.pose.theta))
        
        # Actualizar pose
        self.pose.x += dx
        self.pose.y += dy
        self.pose.theta = (self.pose.theta + dtheta) % (2 * math.pi)
    
    def __str__(self) -> str:
        """Representación en string del robot."""
        return f"SimulatedDifferentialRobot(pose={self.pose}, left={self.left_speed:.3f}, right={self.right_speed:.3f})"


class SimulatedAckermannRobot(IAckermannRobot):
    """
    Implementación simulada de robot Ackermann.
    
    Usa la funcionalidad común de IRobotBase y solo implementa
    la limitación de ángulo y cinemática específica.
    """
    
    def __init__(self, wheelbase: float = 0.25, max_steering_angle: float = math.pi/6):
        """
        Inicializa el robot Ackermann simulado.
        
        Args:
            wheelbase: Distancia entre ejes (metros)
            max_steering_angle: Ángulo máximo de dirección (radianes)
        """
        super().__init__()
        self.wheelbase = wheelbase
        self.max_steering_angle = max_steering_angle
        
    def set_steering_angle(self, angle: float) -> None:
        """Establece el ángulo de dirección con limitación."""
        # Limitar el ángulo dentro del rango permitido (especialización de Ackermann)
        self.steering_angle = max(-self.max_steering_angle, 
                                min(self.max_steering_angle, angle))
    
    def step(self, dt: float) -> None:
        """
        Implementación simulada del paso de tiempo.
        
        Usa el modelo cinemático de Ackermann para actualizar la pose.
        
        Args:
            dt: Paso de tiempo en segundos
        """
        if abs(self.drive_speed) < 1e-6:
            return  # No hay movimiento
        
        # Obtener pose actual
        x, y, theta = self.pose.to_tuple()
        
        # Modelo cinemático de Ackermann
        if abs(self.steering_angle) < 1e-6:
            # Movimiento recto
            dx = self.drive_speed * math.cos(theta) * dt
            dy = self.drive_speed * math.sin(theta) * dt
            dtheta = 0.0
        else:
            # Movimiento con giro
            R = self.wheelbase / math.tan(self.steering_angle)
            omega = self.drive_speed / R
            dtheta = omega * dt
            dx = R * (math.sin(theta + dtheta) - math.sin(theta))
            dy = R * (-math.cos(theta + dtheta) + math.cos(theta))
        
        # Actualizar pose
        self.pose.x += dx
        self.pose.y += dy
        self.pose.theta = (self.pose.theta + dtheta) % (2 * math.pi)
    
    def __str__(self) -> str:
        """Representación en string del robot."""
        return f"SimulatedAckermannRobot(pose={self.pose}, steering={self.steering_angle:.3f}, speed={self.drive_speed:.3f})"


class BasicNavigationExample:
    """Ejemplo de navegación básica punto a punto."""
    
    def __init__(self, robot: IRobotBase):
        """
        Inicializa el ejemplo.
        
        Args:
            robot: Instancia del robot que implementa IRobotBase (interfaz Ackermann unificada)
        """
        self.robot = robot
        self.controller = NavigationController(max_speed=0.5, linear_gain=1.5, angular_gain=1.0)
        self.visualizer = DataVisualizer(80, 24)
        
        # Lista de objetivos a visitar (waypoints simples)
        self.waypoints = [
            (1.0, 0.0),
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
                self.controller.set_target(target[0], target[1], tol=0.05)
                
                # Obtener pose actual del robot
                current_pose = self.robot.get_pose()
                current_x, current_y, current_theta = current_pose.to_tuple()
                
                # Usar NavigationController para calcular comandos Ackermann
                drive_speed, steering_angle = self.controller.calculate_control_commands(
                    current_x, current_y, current_theta
                )
                
                # Aplicar comandos usando interfaz Ackermann unificada
                self.robot.set_drive_speed(drive_speed)
                self.robot.set_steering_angle(steering_angle)
                self.robot.step(time_step)
                
                # Verificar si llegamos al objetivo usando NavigationController
                if self.controller.is_target_reached(current_x, current_y):
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
            dx = target[0] - current_x
            dy = target[1] - current_y
            distance = math.sqrt(dx*dx + dy*dy)
            
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
