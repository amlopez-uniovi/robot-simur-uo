"""
Herramientas de visualización para datos de robots.

Ejemplo:
    >>> viz = DataVisualizer(60, 20)
    >>> viz.set_bounds(-5, 5, -5, 5)
    >>> viz.draw_point(0, 0, '*')
    >>> print(viz.render())
"""

from typing import List, Tuple, Dict, Any, Optional


class DataVisualizer:
    """
    Clase para visualizar datos de robots (versión simplificada para consola).
    """
    
    def __init__(self, width: int = 80, height: int = 24):
        """
        Inicializa el visualizador.

        Args:
            width (int): Ancho del área de visualización en caracteres.
            height (int): Alto del área de visualización en caracteres.
        """
        self.width = width
        self.height = height
        self.grid = [[' ' for _ in range(width)] for _ in range(height)]
    
    def clear(self):
        """
        Limpia la grilla de visualización.
        """
        self.grid = [[' ' for _ in range(self.width)] for _ in range(self.height)]
    
    def set_bounds(self, min_x: float, max_x: float, min_y: float, max_y: float):
        """
        Establece los límites del mundo a visualizar.

        Args:
            min_x (float): Límite mínimo en X.
            max_x (float): Límite máximo en X.
            min_y (float): Límite mínimo en Y.
            max_y (float): Límite máximo en Y.
        """
        self.min_x = min_x
        self.max_x = max_x
        self.min_y = min_y
        self.max_y = max_y
        self.scale_x = (self.width - 1) / (max_x - min_x) if max_x > min_x else 1
        self.scale_y = (self.height - 1) / (max_y - min_y) if max_y > min_y else 1
    
    def world_to_screen(self, x: float, y: float) -> Tuple[int, int]:
        """
        Convierte coordenadas del mundo a coordenadas de pantalla.

        Args:
            x (float): Coordenada X del mundo.
            y (float): Coordenada Y del mundo.

        Returns:
            Tuple[int, int]: Coordenadas de pantalla (col, row).
        """
        if not hasattr(self, 'scale_x'):
            # Usar valores por defecto si no se han establecido límites
            self.set_bounds(-5, 5, -5, 5)
        
        screen_x = int((x - self.min_x) * self.scale_x)
        screen_y = int((self.max_y - y) * self.scale_y)  # Invertir Y
        
        # Asegurar que esté dentro de los límites
        screen_x = max(0, min(self.width - 1, screen_x))
        screen_y = max(0, min(self.height - 1, screen_y))
        
        return screen_x, screen_y
    
    def draw_point(self, x: float, y: float, char: str = '*'):
        """
        Dibuja un punto en las coordenadas del mundo.

        Args:
            x (float): Coordenada X.
            y (float): Coordenada Y.
            char (str): Carácter a dibujar.
        """
        screen_x, screen_y = self.world_to_screen(x, y)
        self.grid[screen_y][screen_x] = char
    
    def draw_robot(self, x: float, y: float, theta: float, char: str = 'R'):
        """
        Dibuja un robot con su orientación.

        Args:
            x (float): Posición X.
            y (float): Posición Y.
            theta (float): Orientación en radianes.
            char (str): Carácter base para el robot.
        """
        import math
        
        # Dibujar el robot
        self.draw_point(x, y, char)
        
        # Dibujar indicador de dirección
        front_x = x + 0.3 * math.cos(theta)
        front_y = y + 0.3 * math.sin(theta)
        self.draw_point(front_x, front_y, '>')
    
    def draw_line(self, x1: float, y1: float, x2: float, y2: float, char: str = '-'):
        """
        Dibuja una línea entre dos puntos.

        Args:
            x1 (float): X inicial.
            y1 (float): Y inicial.
            x2 (float): X final.
            y2 (float): Y final.
            char (str): Carácter para la línea.
        """
        import math
        
        # Algoritmo de línea simple
        distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        num_points = max(int(distance * 20), 1)  # Densidad de puntos
        
        for i in range(num_points + 1):
            t = i / num_points if num_points > 0 else 0
            x = x1 + t * (x2 - x1)
            y = y1 + t * (y2 - y1)
            self.draw_point(x, y, char)
    
    def draw_circle(self, center_x: float, center_y: float, radius: float, char: str = 'o'):
        """
        Dibuja un círculo.

        Args:
            center_x (float): Centro X.
            center_y (float): Centro Y.
            radius (float): Radio.
            char (str): Carácter para el círculo.
        """
        import math
        
        # Dibujar puntos en el perímetro del círculo
        num_points = max(int(2 * math.pi * radius * 10), 8)
        
        for i in range(num_points):
            angle = 2 * math.pi * i / num_points
            x = center_x + radius * math.cos(angle)
            y = center_y + radius * math.sin(angle)
            self.draw_point(x, y, char)
    
    def draw_lidar_scan(self, robot_x: float, robot_y: float, 
                       ranges: List[float], angles: List[float],
                       max_range: float = 5.0):
        """
        Dibuja un escaneo LiDAR.

        Args:
            robot_x (float): Posición X del robot.
            robot_y (float): Posición Y del robot.
            ranges (List[float]): Distancias medidas.
            angles (List[float]): Ángulos de medición.
            max_range (float): Rango máximo del sensor.
        """
        import math
        
        for range_val, angle in zip(ranges, angles):
            if range_val > 0 and range_val < max_range:
                # Punto de obstáculo
                obs_x = robot_x + range_val * math.cos(angle)
                obs_y = robot_y + range_val * math.sin(angle)
                self.draw_point(obs_x, obs_y, '.')
    
    def draw_path(self, waypoints: List[Tuple[float, float]], char: str = '+'):
        """
        Dibuja una ruta como serie de waypoints.

        Args:
            waypoints (List[Tuple[float, float]]): Lista de puntos (x, y).
            char (str): Carácter para los waypoints.
        """
        for i, (x, y) in enumerate(waypoints):
            # Numerar waypoints si son pocos
            if len(waypoints) <= 10:
                display_char = str(i) if i < 10 else char
            else:
                display_char = char
            self.draw_point(x, y, display_char)
            
            # Conectar con líneas
            if i > 0:
                prev_x, prev_y = waypoints[i-1]
                self.draw_line(prev_x, prev_y, x, y, '.')
    
    def add_text(self, x: int, y: int, text: str):
        """
        Añade texto en coordenadas de pantalla.

        Args:
            x (int): Posición X en caracteres.
            y (int): Posición Y en caracteres.
            text (str): Texto a mostrar.
        """
        if 0 <= y < self.height:
            for i, char in enumerate(text):
                if 0 <= x + i < self.width:
                    self.grid[y][x + i] = char
    
    def render(self) -> str:
        """
        Renderiza la visualización como string.

        Returns:
            str: String con la visualización.
        """
        lines = []
        
        # Añadir borde superior
        lines.append('┌' + '─' * self.width + '┐')
        
        # Añadir filas con bordes laterales
        for row in self.grid:
            line = '│' + ''.join(row) + '│'
            lines.append(line)
        
        # Añadir borde inferior
        lines.append('└' + '─' * self.width + '┘')
        
        return '\n'.join(lines)
    
    def print_visualization(self):
        """
        Imprime la visualización en consola.
        """
        print(self.render())
    
    def create_sensor_chart(self, sensor_data: Dict[str, List[float]], 
                          titles: List[str] = None) -> str:
        """
        Crea un gráfico simple de barras para datos de sensores.

        Args:
            sensor_data (Dict[str, List[float]]): Diccionario con datos de sensores.
            titles (List[str], optional): Títulos para los sensores.

        Returns:
            str: String con el gráfico.
        """
        if not sensor_data:
            return "No hay datos de sensores"
        
        lines = []
        lines.append("Datos de Sensores:")
        lines.append("─" * 40)
        
        for sensor_name, values in sensor_data.items():
            if values:
                avg_val = sum(values) / len(values)
                max_val = max(values)
                min_val = min(values)
                
                # Crear barra simple
                bar_length = int(avg_val * 20)  # Escalar a 20 caracteres max
                bar = '█' * bar_length + '░' * (20 - bar_length)
                
                lines.append(f"{sensor_name:12}: [{bar}] {avg_val:.2f}")
                lines.append(f"{'':12}  Min: {min_val:.2f}, Max: {max_val:.2f}")
        
        return '\n'.join(lines)
    
    def save_to_file(self, filename: str):
        """
        Guarda la visualización actual a un archivo.

        Args:
            filename (str): Nombre del archivo.
        """
        try:
            with open(filename, 'w', encoding='utf-8') as f:
                f.write(self.render())
        except IOError as e:
            print(f"Error guardando visualización: {e}")


def create_simple_map(robot_pos: Tuple[float, float], 
                     obstacles: List[Tuple[float, float]], 
                     goal: Tuple[float, float] = None,
                     bounds: Tuple[float, float, float, float] = (-5, 5, -5, 5)) -> str:
    """
    Crea un mapa simple con robot, obstáculos y objetivo.

    Args:
        robot_pos (Tuple[float, float]): Posición del robot (x, y).
        obstacles (List[Tuple[float, float]]): Lista de obstáculos (x, y).
        goal (Tuple[float, float], optional): Posición objetivo opcional (x, y).
        bounds (Tuple[float, float, float, float], optional): Límites del mapa (min_x, max_x, min_y, max_y).

    Returns:
        str: String con el mapa.
    """
    viz = DataVisualizer(60, 20)
    viz.set_bounds(*bounds)
    # Dibujar robot
    viz.draw_robot(robot_pos[0], robot_pos[1], 0, 'R')
    # Dibujar obstáculos
    for obs_x, obs_y in obstacles:
        viz.draw_point(obs_x, obs_y, '#')
    # Dibujar objetivo
    if goal:
        viz.draw_point(goal[0], goal[1], 'G')
    return viz.render()
