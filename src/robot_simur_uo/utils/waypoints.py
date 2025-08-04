"""
Clase para gestión y generación de waypoints.
Proporciona métodos para crear diferentes patrones de waypoints.
"""

import math
from typing import List, Tuple


class Waypoints:
    """
    Clase para gestionar y generar listas de waypoints (puntos de ruta).
    
    Permite crear patrones geométricos predefinidos o usar listas personalizadas
    de puntos para navegación de robots.
    """
    
    def __init__(self):
        """Inicializa la clase Waypoints con una lista vacía."""
        self._waypoints: List[Tuple[float, float]] = []
    
    def set_waypoints(self, waypoints: List[Tuple[float, float]]) -> 'Waypoints':
        """
        Establece una lista personalizada de waypoints.
        
        Args:
            waypoints: Lista de tuplas (x, y) representando los puntos de ruta
            
        Returns:
            Self para permitir method chaining
            
        Raises:
            ValueError: Si la lista está vacía
        """
        if not waypoints:
            raise ValueError("La lista de waypoints no puede estar vacía")
        
        self._waypoints = waypoints.copy()  # Copia para evitar modificaciones externas
        return self
    
    def create_square_route(self, center_x: float = 0.0, center_y: float = 0.0, 
                          size: float = 2.0) -> 'Waypoints':
        """
        Crea una ruta cuadrada predefinida.
        
        Args:
            center_x: Centro X del cuadrado
            center_y: Centro Y del cuadrado
            size: Tamaño del lado del cuadrado en metros
            
        Returns:
            Self para permitir method chaining
        """
        half_size = size / 2
        self._waypoints = [
            (center_x - half_size, center_y - half_size),  # Esquina inferior izquierda
            (center_x + half_size, center_y - half_size),  # Esquina inferior derecha
            (center_x + half_size, center_y + half_size),  # Esquina superior derecha
            (center_x - half_size, center_y + half_size),  # Esquina superior izquierda
        ]
        
        print(f"🔷 Ruta cuadrada creada: {len(self._waypoints)} waypoints")
        print(f"   Centro: ({center_x:.2f}, {center_y:.2f})")
        print(f"   Tamaño: {size:.2f}m x {size:.2f}m")
        
        return self
    
    def create_circular_route(self, center_x: float = 0.0, center_y: float = 0.0,
                            radius: float = 1.0, num_points: int = 8) -> 'Waypoints':
        """
        Crea una ruta circular predefinida.
        
        Args:
            center_x: Centro X del círculo
            center_y: Centro Y del círculo
            radius: Radio del círculo en metros
            num_points: Número de puntos distribuidos en el círculo
            
        Returns:
            Self para permitir method chaining
        """
        if num_points < 3:
            raise ValueError("El número mínimo de puntos es 3")
        
        self._waypoints = []
        for i in range(num_points):
            angle = 2 * math.pi * i / num_points
            x = center_x + radius * math.cos(angle)
            y = center_y + radius * math.sin(angle)
            self._waypoints.append((x, y))
        
        print(f"⭕ Ruta circular creada: {len(self._waypoints)} waypoints")
        print(f"   Centro: ({center_x:.2f}, {center_y:.2f})")
        print(f"   Radio: {radius:.2f}m")
        
        return self
    
    def create_rectangular_route(self, center_x: float = 0.0, center_y: float = 0.0,
                               width: float = 2.0, height: float = 1.0) -> 'Waypoints':
        """
        Crea una ruta rectangular predefinida.
        
        Args:
            center_x: Centro X del rectángulo
            center_y: Centro Y del rectángulo
            width: Ancho del rectángulo en metros
            height: Alto del rectángulo en metros
            
        Returns:
            Self para permitir method chaining
        """
        half_width = width / 2
        half_height = height / 2
        
        self._waypoints = [
            (center_x - half_width, center_y - half_height),  # Esquina inferior izquierda
            (center_x + half_width, center_y - half_height),  # Esquina inferior derecha
            (center_x + half_width, center_y + half_height),  # Esquina superior derecha
            (center_x - half_width, center_y + half_height),  # Esquina superior izquierda
        ]
        
        print(f"📐 Ruta rectangular creada: {len(self._waypoints)} waypoints")
        print(f"   Centro: ({center_x:.2f}, {center_y:.2f})")
        print(f"   Dimensiones: {width:.2f}m x {height:.2f}m")
        
        return self
    
    def create_line_route(self, start_x: float, start_y: float, 
                         end_x: float, end_y: float, num_points: int = 5) -> 'Waypoints':
        """
        Crea una ruta lineal entre dos puntos.
        
        Args:
            start_x: Coordenada X del punto inicial
            start_y: Coordenada Y del punto inicial
            end_x: Coordenada X del punto final
            end_y: Coordenada Y del punto final
            num_points: Número de waypoints intermedios a generar
            
        Returns:
            Self para permitir method chaining
        """
        if num_points < 2:
            raise ValueError("El número mínimo de puntos es 2")
        
        self._waypoints = []
        for i in range(num_points):
            t = i / (num_points - 1)  # Factor de interpolación de 0 a 1
            x = start_x + t * (end_x - start_x)
            y = start_y + t * (end_y - start_y)
            self._waypoints.append((x, y))
        
        print(f"📏 Ruta lineal creada: {len(self._waypoints)} waypoints")
        print(f"   Desde: ({start_x:.2f}, {start_y:.2f})")
        print(f"   Hasta: ({end_x:.2f}, {end_y:.2f})")
        
        return self
    
    def add_waypoint(self, x: float, y: float, index: int = None) -> 'Waypoints':
        """
        Añade un waypoint a la lista existente.
        
        Args:
            x: Coordenada X del waypoint
            y: Coordenada Y del waypoint
            index: Índice donde insertar (None = al final)
            
        Returns:
            Self para permitir method chaining
        """
        if index is None:
            self._waypoints.append((x, y))
        else:
            self._waypoints.insert(index, (x, y))
        
        print(f"➕ Waypoint añadido: ({x:.2f}, {y:.2f}) - Total: {len(self._waypoints)}")
        return self
    
    def remove_waypoint(self, index: int) -> 'Waypoints':
        """
        Elimina un waypoint de la lista.
        
        Args:
            index: Índice del waypoint a eliminar
            
        Returns:
            Self para permitir method chaining
            
        Raises:
            IndexError: Si el índice está fuera de rango
        """
        if 0 <= index < len(self._waypoints):
            removed = self._waypoints.pop(index)
            print(f"➖ Waypoint eliminado: ({removed[0]:.2f}, {removed[1]:.2f})")
        else:
            raise IndexError(f"Índice {index} fuera de rango. Waypoints disponibles: {len(self._waypoints)}")
        
        return self
    
    def get_waypoints(self) -> List[Tuple[float, float]]:
        """
        Obtiene la lista actual de waypoints.
        
        Returns:
            Lista de tuplas (x, y) representando los waypoints
        """
        return self._waypoints.copy()  # Copia para evitar modificaciones externas
    
    def clear(self) -> 'Waypoints':
        """
        Limpia la lista de waypoints.
        
        Returns:
            Self para permitir method chaining
        """
        self._waypoints.clear()
        print("🗑️ Lista de waypoints limpiada")
        return self
    
    def count(self) -> int:
        """
        Obtiene el número de waypoints en la lista.
        
        Returns:
            Número de waypoints
        """
        return len(self._waypoints)
    
    def is_empty(self) -> bool:
        """
        Verifica si la lista de waypoints está vacía.
        
        Returns:
            True si está vacía, False en caso contrario
        """
        return len(self._waypoints) == 0
    
    def print_waypoints(self) -> 'Waypoints':
        """
        Imprime la lista actual de waypoints de forma legible.
        
        Returns:
            Self para permitir method chaining
        """
        if self.is_empty():
            print("📍 Lista de waypoints vacía")
        else:
            print(f"📍 Waypoints actuales ({len(self._waypoints)} puntos):")
            for i, (x, y) in enumerate(self._waypoints, 1):
                print(f"   {i}: ({x:.2f}, {y:.2f})")
        
        return self
    
    def __len__(self) -> int:
        """Permite usar len() con la clase."""
        return len(self._waypoints)
    
    def __getitem__(self, index: int) -> Tuple[float, float]:
        """Permite acceso por índice como una lista."""
        return self._waypoints[index]
    
    def __iter__(self):
        """Permite iterar sobre los waypoints."""
        return iter(self._waypoints)
    
    def __str__(self) -> str:
        """Representación en string de la clase."""
        return f"Waypoints({len(self._waypoints)} puntos)"
    
    def __repr__(self) -> str:
        """Representación detallada de la clase."""
        return f"Waypoints(waypoints={self._waypoints})"


if __name__ == "__main__":
    """Ejemplo de uso de la clase Waypoints."""
    print("=== DEMO DE LA CLASE WAYPOINTS ===")
    
    # Crear instancia
    waypoints = Waypoints()
    
    # Crear ruta cuadrada
    print("\n🔷 CREANDO RUTA CUADRADA:")
    waypoints.create_square_route(center_x=0, center_y=0, size=2.0)
    waypoints.print_waypoints()
    
    # Crear ruta circular
    print("\n⭕ CREANDO RUTA CIRCULAR:")
    waypoints.create_circular_route(center_x=0, center_y=0, radius=1.5, num_points=6)
    waypoints.print_waypoints()
    
    # Crear waypoints personalizados
    print("\n📍 WAYPOINTS PERSONALIZADOS:")
    custom_points = [(0, 0), (1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0)]
    waypoints.set_waypoints(custom_points)
    waypoints.print_waypoints()
    
    # Añadir waypoint
    print("\n➕ AÑADIENDO WAYPOINT:")
    waypoints.add_waypoint(0.5, 0.5)
    waypoints.print_waypoints()
    
    # Crear ruta lineal
    print("\n📏 CREANDO RUTA LINEAL:")
    waypoints.create_line_route(-2, -2, 2, 2, num_points=5)
    waypoints.print_waypoints()
    
    print(f"\n✅ Demo completada. Total de waypoints: {len(waypoints)}")
