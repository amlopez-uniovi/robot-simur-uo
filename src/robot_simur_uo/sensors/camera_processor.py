"""
Procesador de imágenes de cámara para robots.
"""

from typing import List, Tuple, Optional, Any


class CameraProcessor:
    """Procesador básico para imágenes de cámara."""
    
    def __init__(self, width: int = 640, height: int = 480):
        """
        Inicializa el procesador de cámara.
        
        Args:
            width: Ancho de imagen en píxeles
            height: Alto de imagen en píxeles
        """
        self.width = width
        self.height = height
    
    def detect_line_center(self, image_data: List[int], 
                          threshold: int = 128) -> Optional[float]:
        """
        Detecta el centro de una línea en la imagen (para seguimiento de líneas).
        
        Args:
            image_data: Datos de imagen como lista de valores de píxeles
            threshold: Umbral para binarización
            
        Returns:
            Posición X del centro de línea normalizada (-1 a 1), o None si no se detecta
        """
        if len(image_data) != self.width * self.height:
            return None
        
        # Analizar la fila central de la imagen
        center_row = self.height // 2
        row_start = center_row * self.width
        row_end = row_start + self.width
        
        # Extraer píxeles de la fila central
        row_pixels = image_data[row_start:row_end]
        
        # Binarizar
        binary_pixels = [1 if pixel < threshold else 0 for pixel in row_pixels]
        
        # Encontrar segmentos de línea
        line_pixels = []
        for i, pixel in enumerate(binary_pixels):
            if pixel == 1:
                line_pixels.append(i)
        
        if not line_pixels:
            return None
        
        # Calcular centro de la línea
        line_center = sum(line_pixels) / len(line_pixels)
        
        # Normalizar a rango [-1, 1]
        normalized_center = (line_center - self.width/2) / (self.width/2)
        
        return normalized_center
    
    def detect_objects_by_color(self, image_data: List[int],
                               target_color_range: Tuple[int, int]) -> List[Tuple[int, int, int]]:
        """
        Detecta objetos por rango de color (versión simplificada).
        
        Args:
            image_data: Datos de imagen (escala de grises)
            target_color_range: Rango de color (min, max)
            
        Returns:
            Lista de objetos detectados (x, y, tamaño)
        """
        min_color, max_color = target_color_range
        objects = []
        
        # Buscar grupos de píxeles en el rango de color
        visited = [False] * len(image_data)
        
        for i in range(len(image_data)):
            if (not visited[i] and 
                min_color <= image_data[i] <= max_color):
                
                # Flood fill para encontrar objeto conectado
                object_pixels = self._flood_fill(image_data, i, visited, target_color_range)
                
                if len(object_pixels) > 10:  # Filtrar objetos muy pequeños
                    # Calcular centro y tamaño
                    x_coords = [p % self.width for p in object_pixels]
                    y_coords = [p // self.width for p in object_pixels]
                    
                    center_x = sum(x_coords) // len(x_coords)
                    center_y = sum(y_coords) // len(y_coords)
                    size = len(object_pixels)
                    
                    objects.append((center_x, center_y, size))
        
        return objects
    
    def _flood_fill(self, image_data: List[int], start_idx: int, 
                   visited: List[bool], color_range: Tuple[int, int]) -> List[int]:
        """
        Flood fill para detectar objetos conectados.
        
        Args:
            image_data: Datos de imagen
            start_idx: Índice de inicio
            visited: Array de píxeles visitados
            color_range: Rango de color objetivo
            
        Returns:
            Lista de índices de píxeles del objeto
        """
        min_color, max_color = color_range
        stack = [start_idx]
        object_pixels = []
        
        while stack:
            idx = stack.pop()
            
            if (idx < 0 or idx >= len(image_data) or 
                visited[idx] or 
                not (min_color <= image_data[idx] <= max_color)):
                continue
            
            visited[idx] = True
            object_pixels.append(idx)
            
            # Agregar vecinos (4-conectividad)
            x = idx % self.width
            y = idx // self.width
            
            # Vecino izquierdo
            if x > 0:
                stack.append(idx - 1)
            # Vecino derecho
            if x < self.width - 1:
                stack.append(idx + 1)
            # Vecino superior
            if y > 0:
                stack.append(idx - self.width)
            # Vecino inferior
            if y < self.height - 1:
                stack.append(idx + self.width)
        
        return object_pixels
    
    def calculate_object_distance(self, object_size: int, 
                                known_size: float = 0.05,
                                focal_length: float = 500.0) -> float:
        """
        Estima distancia a un objeto basándose en su tamaño en la imagen.
        
        Args:
            object_size: Tamaño del objeto en píxeles
            known_size: Tamaño real conocido del objeto en metros
            focal_length: Longitud focal de la cámara en píxeles
            
        Returns:
            Distancia estimada en metros
        """
        if object_size <= 0:
            return float('inf')
        
        # Fórmula básica de visión por computadora
        distance = (known_size * focal_length) / object_size
        
        return distance
    
    def get_image_brightness(self, image_data: List[int]) -> float:
        """
        Calcula el brillo promedio de la imagen.
        
        Args:
            image_data: Datos de imagen
            
        Returns:
            Brillo promedio (0-255)
        """
        if not image_data:
            return 0.0
        
        return sum(image_data) / len(image_data)
    
    def normalize_coordinates(self, x: int, y: int) -> Tuple[float, float]:
        """
        Normaliza coordenadas de píxeles a rango [-1, 1].
        
        Args:
            x: Coordenada X en píxeles
            y: Coordenada Y en píxeles
            
        Returns:
            Coordenadas normalizadas (x_norm, y_norm)
        """
        x_norm = (x - self.width/2) / (self.width/2)
        y_norm = (y - self.height/2) / (self.height/2)
        
        return x_norm, y_norm
