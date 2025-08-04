"""
Controlador de navegación con estrategia Look-Ahead.
Implementa una navegación más suave anticipando la trayectoria futura.
"""

import math
from typing import Tuple
from .navigation import NavigationController


class NavigationLookAhead(NavigationController):
    """
    Controlador de navegación que implementa una estrategia de look-ahead.
    
    La estrategia look-ahead calcula un punto objetivo virtual adelantado
    en la dirección del movimiento para producir comandos de control más suaves
    y anticipar mejor la trayectoria hacia el objetivo.
    """
    
    def __init__(self, 
                 linear_gain: float = 2.0, 
                 steering_gain: float = 2.0,
                 lookahead_distance: float = 0.3,
                 min_lookahead: float = 0.1,
                 max_lookahead: float = 0.8):
        """
        Inicializa el controlador de navegación con look-ahead.
        
        Args:
            linear_gain: Ganancia lineal para velocidad
            steering_gain: Ganancia angular para velocidad
            lookahead_distance: Distancia base de look-ahead (metros)
            min_lookahead: Distancia mínima de look-ahead (metros)
            max_lookahead: Distancia máxima de look-ahead (metros)
        """
        super().__init__(linear_gain, steering_gain)
        
        self.lookahead_distance = lookahead_distance
        self.min_lookahead = min_lookahead
        self.max_lookahead = max_lookahead
        
        print(f"🎯 NavigationLookAhead inicializado:")
        print(f"   Look-ahead base: {self.lookahead_distance:.2f}m")
        print(f"   Rango: {self.min_lookahead:.2f}m - {self.max_lookahead:.2f}m")
    
    def calculate_lookahead_point(self, current_x: float, current_y: float, 
                                current_angle: float, current_speed: float = 0.0) -> Tuple[float, float]:
        """
        Calcula el punto de look-ahead usando seguimiento de trayectorias.
        
        Implementa el algoritmo de seguimiento de trayectorias con factor de lookahead:
        1. vector_direccion = (x_siguiente - x_actual, y_siguiente - y_actual)
        2. magnitud = sqrt(vector_direccion[0]² + vector_direccion[1]²)
        3. direccion_normalizada = vector_direccion / magnitud
        4. punto_anticipado = posicion_actual + lookahead_factor * direccion_normalizada
        
        Args:
            current_x: Posición X actual
            current_y: Posición Y actual
            current_angle: Ángulo actual del robot (no usado en esta implementación)
            current_speed: Velocidad actual del robot (opcional, para lookahead adaptativo)
            
        Returns:
            Tuple con (x_lookahead, y_lookahead)
        """
        if self.target_x is None or self.target_y is None:
            return current_x, current_y
        
        # 1. Calcular vector dirección: (x_siguiente - x_actual, y_siguiente - y_actual)
        vector_direccion = (self.target_x - current_x, self.target_y - current_y)
        
        # 2. Calcular magnitud: sqrt(vector_direccion[0]² + vector_direccion[1]²)
        magnitud = math.sqrt(vector_direccion[0]**2 + vector_direccion[1]**2)
        
        # Si estamos muy cerca del objetivo, devolver el objetivo directamente
        if magnitud < 0.01:  # Evitar división por cero
            return self.target_x, self.target_y
        
        # 3. Calcular dirección normalizada: vector_direccion / magnitud
        direccion_normalizada = (vector_direccion[0] / magnitud, vector_direccion[1] / magnitud)
        
        # 4. Calcular punto anticipado usando el factor de lookahead
        # punto_anticipado = (x_actual + lookahead_factor * direccion_normalizada[0], 
        #                     y_actual + lookahead_factor * direccion_normalizada[1])
        
        # Usar lookahead_distance como el lookahead_factor
        lookahead_factor = self.lookahead_distance
        
        # Adaptar el factor según la velocidad (opcional)
        #if current_speed > 0:
        #    lookahead_factor = self.lookahead_distance + (current_speed * 0.2)
        
        # Aplicar límites al factor de lookahead
        lookahead_factor = max(self.min_lookahead, min(self.max_lookahead, lookahead_factor))
        
        # Si el factor de lookahead es mayor que la distancia al objetivo,
        # usar el objetivo directamente para evitar overshoot
        if lookahead_factor >= magnitud:
            return self.target_x, self.target_y
        
        # Calcular punto anticipado
        punto_anticipado_x = current_x + lookahead_factor * direccion_normalizada[0]
        punto_anticipado_y = current_y + lookahead_factor * direccion_normalizada[1]
        
        return punto_anticipado_x, punto_anticipado_y
    
    def calculate_control_commands(self, current_x: float, current_y: float, 
                                 current_angle: float, current_speed: float = 0.0) -> Tuple[float, float]:
        """
        Calcula comandos de control usando estrategia look-ahead.
        
        La estrategia look-ahead mejora la navegación al:
        1. Calcular un punto objetivo virtual adelantado
        2. Dirigirse hacia ese punto en lugar del objetivo final
        3. Producir movimientos más suaves y anticipativos
        
        Args:
            current_x: Posición X actual
            current_y: Posición Y actual  
            current_angle: Ángulo actual del robot
            current_speed: Velocidad actual del robot (opcional)
            
        Returns:
            Tuple con (velocidad_lineal, velocidad_de_dirección)
        """
        if self.target_x is None or self.target_y is None:
            return 0.0, 0.0
        
        # Verificar si hemos alcanzado el objetivo final
        final_distance = math.sqrt(
            (self.target_x - current_x)**2 + (self.target_y - current_y)**2
        )
        
        if final_distance < self.tolerance:
            return 0.0, 0.0
        
        # Calcular punto de look-ahead
        lookahead_x, lookahead_y = self.calculate_lookahead_point(
            current_x, current_y, current_angle, current_speed
        )
        
        # Calcular vector hacia el punto de look-ahead
        dx = lookahead_x - current_x
        dy = lookahead_y - current_y
        
        # Calcular ángulo hacia el punto de look-ahead
        target_angle = math.atan2(dy, dx)
        
        # Calcular diferencia angular
        angle_diff = math.atan2(
            math.sin(target_angle - current_angle), 
            math.cos(target_angle - current_angle)
        )
        
        # Distancia al punto de look-ahead
        lookahead_distance = math.sqrt(dx**2 + dy**2)
        
        # Control de velocidad: más conservador cerca del objetivo
        speed_factor = min(1.0, final_distance / self.lookahead_distance)
        drive_speed = lookahead_distance * self.linear_gain * speed_factor
        
        # Control angular suavizado
        steering_speed = angle_diff * self.steering_gain
        
        # Reducir velocidad angular cuando vamos rápido hacia adelante
        if abs(angle_diff) > math.pi/4:  # Más de 45 grados de error
            drive_speed *= 0.5  # Reducir velocidad para girar mejor
        
        return drive_speed, steering_speed
    
    def set_lookahead_parameters(self, distance: float, min_dist: float = None, max_dist: float = None):
        """
        Actualiza los parámetros de look-ahead dinámicamente.
        
        Args:
            distance: Nueva distancia base de look-ahead
            min_dist: Nueva distancia mínima (opcional)
            max_dist: Nueva distancia máxima (opcional)
        """
        self.lookahead_distance = distance
        
        if min_dist is not None:
            self.min_lookahead = min_dist
            
        if max_dist is not None:
            self.max_lookahead = max_dist
        
        print(f"🔧 Parámetros look-ahead actualizados:")
        print(f"   Base: {self.lookahead_distance:.2f}m")
        print(f"   Rango: {self.min_lookahead:.2f}m - {self.max_lookahead:.2f}m")
    
    def get_debug_info(self, current_x: float, current_y: float, 
                      current_angle: float, current_speed: float = 0.0) -> dict:
        """
        Obtiene información de depuración del controlador look-ahead.
        
        Args:
            current_x: Posición X actual
            current_y: Posición Y actual
            current_angle: Ángulo actual del robot
            current_speed: Velocidad actual del robot
            
        Returns:
            Diccionario con información de depuración
        """
        if self.target_x is None or self.target_y is None:
            return {}
        
        lookahead_x, lookahead_y = self.calculate_lookahead_point(
            current_x, current_y, current_angle, current_speed
        )
        
        final_distance = math.sqrt(
            (self.target_x - current_x)**2 + (self.target_y - current_y)**2
        )
        
        lookahead_distance = math.sqrt(
            (lookahead_x - current_x)**2 + (lookahead_y - current_y)**2
        )
        
        return {
            'target_position': (self.target_x, self.target_y),
            'lookahead_position': (lookahead_x, lookahead_y),
            'distance_to_target': final_distance,
            'distance_to_lookahead': lookahead_distance,
            'current_lookahead_params': {
                'base': self.lookahead_distance,
                'min': self.min_lookahead,
                'max': self.max_lookahead
            }
        }


if __name__ == "__main__":
    """Ejemplo de uso del controlador NavigationLookAhead."""
    print("=== NavigationLookAhead Demo ===")
    
    # Crear controlador con look-ahead
    controller = NavigationLookAhead(
        linear_gain=1.0, 
        steering_gain=1.5,
        lookahead_distance=0.8,
        min_lookahead=0.2,
        max_lookahead=1.5
    )
    
    # Establecer objetivo
    controller.set_target(3.0, 2.0, tol=0.15)
    
    # Estado inicial del robot
    current_x, current_y, current_angle = 0.0, 0.0, 0.0
    current_speed = 0.0
    
    print(f"\n🎯 Objetivo: ({controller.target_x}, {controller.target_y})")
    print("=" * 60)
    
    # Simulación
    for i in range(20):
        if controller.is_target_reached(current_x, current_y):
            print(f"✅ ¡Objetivo alcanzado en el paso {i+1}!")
            break
            
        # Obtener comandos de control
        drive_speed, steering_speed = controller.calculate_control_commands(
            current_x, current_y, current_angle, current_speed
        )
        
        # Información de depuración
        debug_info = controller.get_debug_info(current_x, current_y, current_angle, current_speed)
        lookahead_pos = debug_info.get('lookahead_position', (0, 0))
        
        print(f"Paso {i+1:2d}: Pos=({current_x:.2f}, {current_y:.2f}) "
              f"Look=({lookahead_pos[0]:.2f}, {lookahead_pos[1]:.2f}) "
              f"V={drive_speed:.2f} ω={steering_speed:.2f}")
        
        # Simulación cinemática simple
        dt = 0.1
        current_speed = drive_speed  # Actualizar velocidad actual
        current_x += drive_speed * math.cos(current_angle) * dt
        current_y += drive_speed * math.sin(current_angle) * dt
        current_angle += steering_speed * dt
        
        # Normalizar ángulo
        current_angle = math.atan2(math.sin(current_angle), math.cos(current_angle))
    
    print("\n🏁 Simulación completada!")
