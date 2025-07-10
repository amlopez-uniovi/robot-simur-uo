"""
Planificador de rutas básico para robots.
"""

import math
from typing import List, Tuple, Optional
from collections import deque


class PathPlanner:
    """Planificador de rutas simple usando A* simplificado."""
    
    def __init__(self, grid_size: float = 0.1):
        """
        Inicializa el planificador de rutas.
        
        Args:
            grid_size: Tamaño de celda de la grilla
        """
        self.grid_size = grid_size
        self.obstacles: List[Tuple[float, float]] = []
        
    def add_obstacle(self, x: float, y: float, radius: float = 0.1):
        """
        Añade un obstáculo circular.
        
        Args:
            x: Coordenada X del obstáculo
            y: Coordenada Y del obstáculo  
            radius: Radio del obstáculo
        """
        self.obstacles.append((x, y, radius))
    
    def clear_obstacles(self):
        """Limpia todos los obstáculos."""
        self.obstacles.clear()
    
    def is_point_free(self, x: float, y: float, safety_margin: float = 0.05) -> bool:
        """
        Verifica si un punto está libre de obstáculos.
        
        Args:
            x: Coordenada X
            y: Coordenada Y
            safety_margin: Margen de seguridad adicional
            
        Returns:
            True si el punto está libre
        """
        for obs_x, obs_y, radius in self.obstacles:
            distance = math.sqrt((x - obs_x)**2 + (y - obs_y)**2)
            if distance < (radius + safety_margin):
                return False
        return True
    
    def plan_straight_line(self, start: Tuple[float, float], 
                          goal: Tuple[float, float],
                          num_waypoints: int = 10) -> List[Tuple[float, float]]:
        """
        Planifica una ruta en línea recta verificando obstáculos.
        
        Args:
            start: Punto de inicio (x, y)
            goal: Punto objetivo (x, y)
            num_waypoints: Número de puntos intermedios a verificar
            
        Returns:
            Lista de waypoints si la ruta es válida, lista vacía si no
        """
        start_x, start_y = start
        goal_x, goal_y = goal
        
        waypoints = []
        
        for i in range(num_waypoints + 1):
            t = i / num_waypoints
            x = start_x + t * (goal_x - start_x)
            y = start_y + t * (goal_y - start_y)
            
            if not self.is_point_free(x, y):
                return []  # Ruta bloqueada
            
            waypoints.append((x, y))
        
        return waypoints
    
    def plan_simple_avoidance(self, start: Tuple[float, float],
                            goal: Tuple[float, float],
                            max_detour: float = 1.0) -> List[Tuple[float, float]]:
        """
        Planifica ruta simple con evitación básica de obstáculos.
        
        Args:
            start: Punto de inicio
            goal: Punto objetivo
            max_detour: Máximo desvío permitido
            
        Returns:
            Lista de waypoints
        """
        # Primero intentar línea recta
        straight_path = self.plan_straight_line(start, goal)
        if straight_path:
            return straight_path
        
        # Si está bloqueado, intentar desvíos laterales
        start_x, start_y = start
        goal_x, goal_y = goal
        
        # Vector dirección
        dx = goal_x - start_x
        dy = goal_y - start_y
        distance = math.sqrt(dx**2 + dy**2)
        
        if distance == 0:
            return [start]
        
        # Vector perpendicular para desvíos
        perp_x = -dy / distance
        perp_y = dx / distance
        
        # Probar desvíos hacia ambos lados
        for side in [-1, 1]:
            detour_distance = 0.2
            while detour_distance <= max_detour:
                # Punto intermedio desviado
                mid_x = start_x + dx/2 + side * perp_x * detour_distance
                mid_y = start_y + dy/2 + side * perp_y * detour_distance
                
                # Verificar ruta: start -> mid -> goal
                path1 = self.plan_straight_line(start, (mid_x, mid_y))
                path2 = self.plan_straight_line((mid_x, mid_y), goal)
                
                if path1 and path2:
                    # Combinar rutas eliminando punto duplicado
                    return path1 + path2[1:]
                
                detour_distance += 0.1
        
        # Si no se encuentra ruta, retornar solo el punto de inicio
        return [start]
    
    def simplify_path(self, waypoints: List[Tuple[float, float]], 
                     tolerance: float = 0.05) -> List[Tuple[float, float]]:
        """
        Simplifica una ruta eliminando waypoints innecesarios.
        
        Args:
            waypoints: Lista de waypoints
            tolerance: Tolerancia para simplificación
            
        Returns:
            Lista simplificada de waypoints
        """
        if len(waypoints) <= 2:
            return waypoints
        
        simplified = [waypoints[0]]
        
        i = 0
        while i < len(waypoints) - 1:
            # Encontrar el waypoint más lejano al que se puede ir en línea recta
            j = len(waypoints) - 1
            while j > i + 1:
                path = self.plan_straight_line(waypoints[i], waypoints[j])
                if path:
                    simplified.append(waypoints[j])
                    i = j
                    break
                j -= 1
            else:
                # Si no se puede ir más lejos, avanzar uno
                i += 1
                if i < len(waypoints):
                    simplified.append(waypoints[i])
        
        return simplified
