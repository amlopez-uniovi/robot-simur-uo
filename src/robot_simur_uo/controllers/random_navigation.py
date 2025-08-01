"""
Controlador de navegación aleatoria que genera objetivos aleatorios y navega hacia ellos.
"""

import random
import math
from typing import Tuple, Optional, List, Dict, Any
from .navigation import NavigationController


class RandomNavigationController:
    """
    Controlador que genera objetivos aleatorios dentro de un espacio de trabajo
    y navega hacia ellos usando NavigationController.
    """
    
    def __init__(self, 
                 workspace_bounds: Tuple[float, float, float, float] = (-2.0, 2.0, -2.0, 2.0),
                 linear_gain: float = 1.0,
                 steering_gain: float = 1.5,
                 goal_tolerance: float = 0.1,
                 sensor_report_interval: int = 50):
        """
        Inicializar el controlador de navegación aleatoria.
        
        Args:
            workspace_bounds: (min_x, max_x, min_y, max_y) límites del espacio de trabajo
            linear_gain: Ganancia para la velocidad lineal
            steering_gain: Ganancia para la velocidad angular
            goal_tolerance: Distancia mínima al objetivo para considerarlo alcanzado
            sensor_report_interval: Intervalo de iteraciones para reportar sensores
        """
        self.workspace_bounds = workspace_bounds
        self.sensor_report_interval = sensor_report_interval
        self.goal_tolerance = goal_tolerance
        
        # Crear controlador de navegación subyacente
        self.nav_controller = NavigationController(
            linear_gain=linear_gain,
            steering_gain=steering_gain
        )
        
        # Estado del controlador
        self.current_goal: Optional[Tuple[float, float]] = None
        self.iteration_count = 0
        self.goals_reached = 0
        self.recent_goals: List[Tuple[float, float]] = []
        
        # Generar primer objetivo
        self._generate_new_goal()
        
    def _generate_new_goal(self) -> None:
        """Generar un nuevo objetivo aleatorio dentro del espacio de trabajo."""
        min_x, max_x, min_y, max_y = self.workspace_bounds
        
        # Generar coordenadas aleatorias
        goal_x = random.uniform(min_x, max_x)
        goal_y = random.uniform(min_y, max_y)
        
        self.current_goal = (goal_x, goal_y)
        
        # Mantener historial de objetivos recientes (máximo 10)
        self.recent_goals.append(self.current_goal)
        if len(self.recent_goals) > 10:
            self.recent_goals.pop(0)
            
        print(f"🎯 Nuevo objetivo generado: ({goal_x:.2f}, {goal_y:.2f})")
        
    def _is_goal_reached(self, current_x: float, current_y: float) -> bool:
        """Verificar si el objetivo actual ha sido alcanzado."""
        if not self.current_goal:
            return False
            
        distance = math.sqrt(
            (self.current_goal[0] - current_x)**2 + 
            (self.current_goal[1] - current_y)**2
        )
        
        return distance <= self.goal_tolerance
        
    def _report_sensors(self, robot, current_x: float, current_y: float, current_angle: float) -> None:
        """Reportar información detallada de sensores y estado."""
        print(f"\n📊 REPORTE DE SENSORES - Iteración {self.iteration_count}")
        print("=" * 50)
        
        # Información de posición
        print(f"📍 Posición actual: ({current_x:.3f}, {current_y:.3f})")
        print(f"🧭 Orientación: {math.degrees(current_angle):.1f}°")
        
        # Información del objetivo
        if self.current_goal:
            goal_distance = math.sqrt(
                (self.current_goal[0] - current_x)**2 + 
                (self.current_goal[1] - current_y)**2
            )
            print(f"🎯 Objetivo actual: ({self.current_goal[0]:.3f}, {self.current_goal[1]:.3f})")
            print(f"📏 Distancia al objetivo: {goal_distance:.3f}m")
            
        # Sensores del robot
        try:
            # GPS
            if hasattr(robot, 'get_gps_position'):
                gps_pos = robot.get_gps_position()
                print(f"🛰️  GPS: ({gps_pos[0]:.3f}, {gps_pos[1]:.3f}, {gps_pos[2]:.3f})")
                
            # Brújula
            if hasattr(robot, 'get_compass_orientation'):
                compass_dir, compass_angle = robot.get_compass_orientation()
                print(f"🧭 Brújula: Dirección={compass_dir}, Ángulo={math.degrees(compass_angle):.1f}°")
                
            # Sensores de distancia si existen
            if hasattr(robot, 'get_distance_sensors'):
                distances = robot.get_distance_sensors()
                if distances:
                    print(f"📡 Sensores distancia: {[f'{d:.3f}' for d in distances[:8]]}")
                    
            # Velocidades de las ruedas si existen
            if hasattr(robot, 'get_wheel_speeds'):
                wheel_speeds = robot.get_wheel_speeds()
                if wheel_speeds:
                    print(f"⚙️  Velocidades ruedas: {[f'{v:.3f}' for v in wheel_speeds]}")
                    
        except Exception as e:
            print(f"⚠️  Error leyendo sensores: {e}")
            
        # Estadísticas de navegación
        print(f"📈 Objetivos alcanzados: {self.goals_reached}")
        print(f"�� Iteraciones totales: {self.iteration_count}")
        print("=" * 50)
        
    def update(self, robot, current_x: float, current_y: float, current_angle: float) -> Tuple[float, float]:
        """
        Actualizar el controlador y obtener comandos de velocidad.
        
        Args:
            robot: Instancia del robot
            current_x: Posición X actual
            current_y: Posición Y actual  
            current_angle: Ángulo actual en radianes
            
        Returns:
            Tupla (velocidad_lineal, velocidad_angular)
        """
        self.iteration_count += 1
        
        # Verificar si se alcanzó el objetivo
        if self._is_goal_reached(current_x, current_y):
            print(f"✅ Objetivo alcanzado en ({self.current_goal[0]:.2f}, {self.current_goal[1]:.2f})")
            self.goals_reached += 1
            self._generate_new_goal()
            
        # Reportar sensores si corresponde
        if self.iteration_count % self.sensor_report_interval == 0:
            self._report_sensors(robot, current_x, current_y, current_angle)
            
        # Usar NavigationController para navegar al objetivo actual
        if self.current_goal:
            # Establecer el objetivo en el NavigationController
            self.nav_controller.set_target(
                self.current_goal[0], 
                self.current_goal[1], 
                tol=self.goal_tolerance
            )
            
            # Obtener comandos de control
            return self.nav_controller.calculate_control_commands(
                current_x, current_y, current_angle
            )
        else:
            return 0.0, 0.0
            
    def get_statistics(self) -> Dict[str, Any]:
        """Obtener estadísticas del controlador."""
        return {
            'iteration_count': self.iteration_count,
            'goals_reached': self.goals_reached,
            'current_goal': self.current_goal,
            'recent_goals': self.recent_goals.copy(),
            'workspace_bounds': self.workspace_bounds,
            'sensor_report_interval': self.sensor_report_interval
        }
        
    def reset(self) -> None:
        """Reiniciar el estado del controlador."""
        self.iteration_count = 0
        self.goals_reached = 0
        self.recent_goals.clear()
        self.nav_controller.set_target(0, 0)  # Reset del NavigationController
        self._generate_new_goal()
