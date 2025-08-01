"""
Controlador de navegación aleatoria para robots.
Genera objetivos aleatorios y navega entre ellos mostrando información detallada de sensores.
"""

import math
import random
from typing import Tuple, Optional, List, Dict, Any, TYPE_CHECKING
from .navigation import NavigationController

if TYPE_CHECKING:
    from ..interfaces.irobot_base import IRobotBase


class RandomNavigationController(NavigationController):
    """Controlador para navegación aleatoria con monitoreo de sensores."""
    
    def __init__(self, 
                 workspace_bounds: Tuple[float, float, float, float] = (-2.0, 2.0, -2.0, 2.0),
                 linear_gain: float = 1.0, 
                 steering_gain: float = 2.0,
                 goal_tolerance: float = 0.15,
                 sensor_report_interval: int = 50):
        """
        Inicializa el controlador de navegación aleatoria.
        
        Args:
            workspace_bounds: Límites del espacio de trabajo (min_x, max_x, min_y, max_y)
            linear_gain: Ganancia lineal para velocidad
            steering_gain: Ganancia angular para velocidad  
            goal_tolerance: Tolerancia para considerar alcanzado el objetivo
            sensor_report_interval: Intervalo en iteraciones para reportar sensores
        """
        # Inicializar la clase padre
        super().__init__(linear_gain, steering_gain)
        
        # Configuración específica de navegación aleatoria
        self.workspace_bounds = workspace_bounds
        self.min_x, self.max_x, self.min_y, self.max_y = workspace_bounds
        self.sensor_report_interval = sensor_report_interval
        self.tolerance = goal_tolerance  # Usar el atributo heredado
        
        # Estado del controlador aleatorio
        self.iteration_count = 0
        self.goals_reached = 0
        
        # Historial de objetivos para evitar repetición inmediata
        self.recent_goals: List[Tuple[float, float]] = []
        self.max_recent_goals = 5
    
    @property
    def current_goal(self) -> Optional[Tuple[float, float]]:
        """Obtiene el objetivo actual como tupla (x, y)."""
        if self.target_x is not None and self.target_y is not None:
            return (self.target_x, self.target_y)
        return None
    
    @current_goal.setter
    def current_goal(self, value: Optional[Tuple[float, float]]):
        """Establece el objetivo actual desde una tupla (x, y)."""
        if value is None:
            self.target_x = None
            self.target_y = None
        else:
            self.target_x, self.target_y = value
    
    def generate_random_goal(self) -> Tuple[float, float]:
        """
        Genera un objetivo aleatorio dentro del espacio de trabajo.
        
        Returns:
            Tupla (x, y) con las coordenadas del objetivo
        """
        attempts = 0
        max_attempts = 50
        
        while attempts < max_attempts:
            # Generar coordenadas aleatorias
            x = random.uniform(self.min_x, self.max_x)
            y = random.uniform(self.min_y, self.max_y)
            goal = (x, y)
            
            # Verificar si está muy cerca de objetivos recientes
            too_close = False
            for recent_goal in self.recent_goals:
                distance = math.sqrt((x - recent_goal[0])**2 + (y - recent_goal[1])**2)
                if distance < 0.5:  # Mínimo 0.5m de separación
                    too_close = True
                    break
            
            if not too_close:
                return goal
                
            attempts += 1
        
        # Si no se pudo generar después de muchos intentos, usar coordenadas básicas
        return (random.uniform(self.min_x, self.max_x), random.uniform(self.min_y, self.max_y))
    
    def set_new_random_goal(self):
        """Establece un nuevo objetivo aleatorio."""
        self.current_goal = self.generate_random_goal()
        # Usar el método heredado en lugar de nav_controller
        self.set_target(self.current_goal[0], self.current_goal[1])
        
        # Agregar a historial de objetivos recientes
        self.recent_goals.append(self.current_goal)
        if len(self.recent_goals) > self.max_recent_goals:
            self.recent_goals.pop(0)
        
        print(f"\n🎯 NUEVO OBJETIVO #{self.goals_reached + 1}: ({self.current_goal[0]:.2f}, {self.current_goal[1]:.2f})")
    
    def update(self, robot: "IRobotBase", current_x: float, current_y: float, current_angle: float) -> Tuple[float, float]:
        """
        Actualiza el controlador y retorna comandos de velocidad.
        
        Args:
            robot: Instancia del robot para obtener datos de sensores
            current_x: Posición X actual
            current_y: Posición Y actual  
            current_angle: Ángulo actual del robot
            
        Returns:
            Tupla (velocidad_lineal, velocidad_angular)
        """
        self.iteration_count += 1
        
        # Si no hay objetivo o se alcanzó el actual, generar uno nuevo
        if (self.current_goal is None or 
            self.is_target_reached(current_x, current_y)):
            
            if self.current_goal is not None:
                self.goals_reached += 1
                print(f"✅ Objetivo alcanzado! Total: {self.goals_reached}")
            
            self.set_new_random_goal()
        
        # Calcular comandos de control usando método heredado
        drive_speed, steering_speed = self.calculate_control_commands(
            current_x, current_y, current_angle
        )
        
        # Reportar estado cada cierto número de iteraciones
        if self.iteration_count % self.sensor_report_interval == 0:
            print(f"\n{'='*80}")
            print(f"📊 REPORTE DETALLADO - Iteración {self.iteration_count}")
            print(f"{'='*80}")
            
            # Información de navegación
            if self.current_goal:
                goal_distance = math.sqrt((self.current_goal[0] - current_x)**2 + 
                                        (self.current_goal[1] - current_y)**2)
                print(f"🧭 NAVEGACIÓN:")
                print(f"   Posición actual: ({current_x:.3f}, {current_y:.3f})")
                print(f"   Objetivo actual: ({self.current_goal[0]:.3f}, {self.current_goal[1]:.3f})")
                print(f"   Distancia al objetivo: {goal_distance:.3f}m")
                print(f"   Ángulo actual: {math.degrees(current_angle):.1f}°")
                print(f"   Velocidad lineal: {drive_speed:.3f}")
                print(f"   Velocidad angular: {steering_speed:.3f}")
                print(f"   Objetivos alcanzados: {self.goals_reached}")
            
            # Información específica del robot usando la función unificada
            robot.log_devices()
            
            print(f"{'='*80}\n")
        
        return drive_speed, steering_speed
    
    def get_statistics(self) -> Dict[str, Any]:
        """
        Obtiene estadísticas del controlador.
        
        Returns:
            Diccionario con estadísticas del controlador
        """
        return {
            'iteration_count': self.iteration_count,
            'goals_reached': self.goals_reached,
            'current_goal': self.current_goal,
            'recent_goals': self.recent_goals.copy()
        }
    
    def get_robot_state(self, robot: "IRobotBase") -> Tuple[float, float, float]:
        """
        Obtiene el estado actual del robot (posición y orientación).
        
        Args:
            robot: Instancia del robot
            
        Returns:
            Tupla (x, y, angle) con la pose actual del robot
        """
        pose = robot.get_pose()
        return pose.x, pose.y, pose.theta
