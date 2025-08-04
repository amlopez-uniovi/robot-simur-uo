"""
Controlador de navegación por waypoints (puntos de ruta).
Navega siguiendo una lista predefinida de puntos en orden secuencial.
"""

import math
import random
from typing import List, Tuple, Optional, Union, TYPE_CHECKING

from .navigation_lookahead import NavigationLookAhead
from ..utils.waypoints import Waypoints

if TYPE_CHECKING:
    from ..interfaces.irobot_base import IRobotBase


class WaypointNavigationController(NavigationLookAhead):
    """
    Controlador que navega siguiendo una lista de waypoints (puntos de ruta) predefinidos.

    Hereda de NavigationLookAhead y añade la funcionalidad de seguir una secuencia
    de puntos específicos en lugar de generar objetivos aleatorios.
    """
    
    def __init__(self, 
                 waypoints: Union[List[Tuple[float, float]], Waypoints],
                 goal_tolerance: float = 0.15,
                 linear_gain: float = 1.0,
                 steering_gain: float = 2.0,
                 max_linear_speed: float = 0.5,
                 max_angular_speed: float = 1.0,
                 cycle_waypoints: bool = True,
                 lookahead_factor: float = 0.2):
        """
        Inicializa el controlador de navegación por waypoints.
        
        Args:
            waypoints: Lista de puntos (x, y) a seguir en orden, o instancia de Waypoints
            goal_tolerance: Tolerancia para considerar alcanzado un waypoint (metros)
            linear_gain: Ganancia del controlador proporcional lineal
            steering_gain: Ganancia del controlador proporcional angular
            max_linear_speed: Velocidad lineal máxima (m/s) - solo informativo
            max_angular_speed: Velocidad angular máxima (rad/s) - solo informativo
            cycle_waypoints: Si True, repite la lista; si False, se detiene al final
            lookahead_factor: Factor de lookahead para navegación suave (metros)
        """
        # Configurar NavigationLookAhead con parámetros optimizados para waypoints
        super().__init__(
            linear_gain=linear_gain, 
            steering_gain=steering_gain,
            lookahead_distance=lookahead_factor,  # Usar el parámetro personalizado
            min_lookahead=max(0.05, lookahead_factor * 0.25),  # Mínimo como 25% del factor
            max_lookahead=min(1.0, lookahead_factor * 2.5)     # Máximo como 250% del factor
        )
        
        # Manejar diferentes tipos de entrada para waypoints
        if isinstance(waypoints, Waypoints):
            waypoints_list = waypoints.get_waypoints()
        else:
            waypoints_list = waypoints
        
        if not waypoints_list:
            raise ValueError("La lista de waypoints no puede estar vacía")
        
        self.waypoints = list(waypoints_list)  # Copia para evitar modificaciones externas
        self.cycle_waypoints = cycle_waypoints
        self.current_waypoint_index = 0
        self.goal_tolerance = goal_tolerance
        self.max_linear_speed = max_linear_speed  # Solo informativo
        self.max_angular_speed = max_angular_speed  # Solo informativo
        
        # Establecer el primer waypoint como objetivo
        self.set_target(
            self.waypoints[0][0], 
            self.waypoints[0][1], 
            tol=self.goal_tolerance
        )
        
        # Estadísticas específicas de waypoints
        self.waypoints_reached = 0
        self.total_cycles_completed = 0
        self.is_route_completed = False
        self._iteration_count = 0  # Contador de iteraciones
        
        print(f"🗺️  WaypointNavigationController inicializado:")
        print(f"   Waypoints: {len(self.waypoints)} puntos")
        print(f"   Modo cíclico: {'Sí' if self.cycle_waypoints else 'No'}")
        print(f"   Primer objetivo: ({self.waypoints[0][0]:.2f}, {self.waypoints[0][1]:.2f})")
    
    def update(self, robot: 'IRobotBase', current_x: float, current_y: float, current_angle: float) -> Tuple[float, float]:
        """
        Actualiza el controlador y devuelve comandos de velocidad.
        
        Args:
            robot: Instancia del robot
            current_x: Posición X actual
            current_y: Posición Y actual
            current_angle: Ángulo actual del robot (radianes)
            
        Returns:
            Tupla (velocidad_avance, velocidad_giro)
        """
        # Incrementar contador de iteraciones
        self._iteration_count += 1
        
        # Verificar si hemos alcanzado el waypoint actual
        if self.is_target_reached(current_x, current_y):
            self._advance_to_next_waypoint()
        
        # Si la ruta está completada y no es cíclica, detener
        if self.is_route_completed:
            return 0.0, 0.0
        
        # Calcular comandos de control usando método heredado
        drive_speed, steering_speed = self.calculate_control_commands(
            current_x, current_y, current_angle
        )
        
        return drive_speed, steering_speed
    
    def _advance_to_next_waypoint(self):
        """Avanza al siguiente waypoint en la lista."""
        self.waypoints_reached += 1
        print(f"✅ Waypoint {self.current_waypoint_index + 1} alcanzado! ({self.waypoints[self.current_waypoint_index][0]:.2f}, {self.waypoints[self.current_waypoint_index][1]:.2f})")
        
        self.current_waypoint_index += 1
        
        # Verificar si hemos completado todos los waypoints
        if self.current_waypoint_index >= len(self.waypoints):
            if self.cycle_waypoints:
                # Reiniciar la secuencia
                self.current_waypoint_index = 0
                self.total_cycles_completed += 1
                print(f"🔄 Ciclo {self.total_cycles_completed} completado. Reiniciando ruta...")
            else:
                # Ruta completada
                self.is_route_completed = True
                print(f"🏁 Ruta completada! Todos los {len(self.waypoints)} waypoints alcanzados.")
                return
        
        # Establecer el siguiente waypoint como objetivo
        next_point = self.waypoints[self.current_waypoint_index]
        self.set_target(next_point[0], next_point[1], tol=self.goal_tolerance)
        print(f"🎯 Nuevo objetivo: Waypoint {self.current_waypoint_index + 1} → ({next_point[0]:.2f}, {next_point[1]:.2f})")
    
    def add_waypoint(self, x: float, y: float, insert_at: Optional[int] = None):
        """
        Añade un nuevo waypoint a la lista.
        
        Args:
            x: Coordenada X del waypoint
            y: Coordenada Y del waypoint
            insert_at: Índice donde insertar (None = al final)
        """
        if insert_at is None:
            self.waypoints.append((x, y))
        else:
            self.waypoints.insert(insert_at, (x, y))
        print(f"➕ Waypoint añadido: ({x:.2f}, {y:.2f}) - Total: {len(self.waypoints)}")
    
    def remove_waypoint(self, index: int):
        """
        Elimina un waypoint de la lista.
        
        Args:
            index: Índice del waypoint a eliminar
        """
        if 0 <= index < len(self.waypoints):
            removed = self.waypoints.pop(index)
            print(f"➖ Waypoint eliminado: ({removed[0]:.2f}, {removed[1]:.2f})")
            
            # Ajustar el índice actual si es necesario
            if index <= self.current_waypoint_index and self.current_waypoint_index > 0:
                self.current_waypoint_index -= 1
        else:
            print(f"❌ Índice {index} fuera de rango. Waypoints disponibles: {len(self.waypoints)}")
    
    def get_remaining_waypoints(self) -> List[Tuple[float, float]]:
        """
        Obtiene la lista de waypoints restantes por visitar en el ciclo actual.
        
        Returns:
            Lista de waypoints pendientes
        """
        if self.is_route_completed:
            return []
        return self.waypoints[self.current_waypoint_index:]
    
    def get_progress_info(self) -> dict:
        """
        Obtiene información detallada del progreso.
        
        Returns:
            Diccionario con información de progreso
        """
        if self.is_route_completed:
            progress_percent = 100.0
        else:
            progress_percent = (self.current_waypoint_index / len(self.waypoints)) * 100
        
        return {
            'current_waypoint_index': self.current_waypoint_index,
            'total_waypoints': len(self.waypoints),
            'waypoints_reached': self.waypoints_reached,
            'total_cycles_completed': self.total_cycles_completed,
            'progress_percent': progress_percent,
            'is_route_completed': self.is_route_completed,
            'cycle_mode': self.cycle_waypoints,
            'current_target': (self.target_x, self.target_y) if not self.is_route_completed else None,
            'remaining_waypoints': self.get_remaining_waypoints()
        }
    
    def reset_route(self):
        """Reinicia la ruta al primer waypoint."""
        self.current_waypoint_index = 0
        self.is_route_completed = False
        
        if self.waypoints:
            self.set_target(
                self.waypoints[0][0], 
                self.waypoints[0][1], 
                tol=self.goal_tolerance
            )
            print(f"🔄 Ruta reiniciada. Objetivo: ({self.waypoints[0][0]:.2f}, {self.waypoints[0][1]:.2f})")
    
    def get_statistics(self) -> dict:
        """
        Obtiene estadísticas extendidas incluyendo información de waypoints.
        
        Returns:
            Diccionario con estadísticas completas
        """
        # Crear estadísticas propias (no hay estadísticas base en NavigationController)
        stats = {
            'iteration_count': getattr(self, '_iteration_count', 0)
        }
        waypoint_stats = self.get_progress_info()
        
        # Combinar estadísticas
        stats.update(waypoint_stats)
        return stats
    
    @property
    def current_waypoint(self) -> Optional[Tuple[float, float]]:
        """Obtiene el waypoint actual (objetivo)."""
        if self.is_route_completed:
            return None
        return self.waypoints[self.current_waypoint_index]
    
    @classmethod
    def create_square_route(cls, center_x: float = 0.0, center_y: float = 0.0, 
                          size: float = 2.0, lookahead_factor: float = 0.2, **kwargs) -> 'WaypointNavigationController':
        """
        Crea una ruta cuadrada predefinida.
        
        Args:
            center_x: Centro X del cuadrado
            center_y: Centro Y del cuadrado
            size: Tamaño del lado del cuadrado
            lookahead_factor: Factor de lookahead para navegación suave (metros)
            **kwargs: Argumentos adicionales para el constructor
            
        Returns:
            Controlador configurado con ruta cuadrada
        """
        waypoints = Waypoints()
        waypoints.create_square_route(center_x, center_y, size)
        
        return cls(waypoints, lookahead_factor=lookahead_factor, **kwargs)
    
    @classmethod
    def create_circular_route(cls, center_x: float = 0.0, center_y: float = 0.0,
                            radius: float = 1.0, num_points: int = 8, 
                            lookahead_factor: float = 0.2, **kwargs) -> 'WaypointNavigationController':
        """
        Crea una ruta circular predefinida.
        
        Args:
            center_x: Centro X del círculo
            center_y: Centro Y del círculo
            radius: Radio del círculo
            num_points: Número de puntos en el círculo
            lookahead_factor: Factor de lookahead para navegación suave (metros)
            **kwargs: Argumentos adicionales para el constructor
            
        Returns:
            Controlador configurado con ruta circular
        """
        waypoints = Waypoints()
        waypoints.create_circular_route(center_x, center_y, radius, num_points)
        
        return cls(waypoints, lookahead_factor=lookahead_factor, **kwargs)
