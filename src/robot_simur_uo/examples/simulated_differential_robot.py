"""
Ejemplo de implementación simulada de robot diferencial.
Implementación completa con toda la funcionalidad.
"""

import math
from typing import Tuple
from ..utils.coordinates import RobotPose
from ..interfaces.idifferential_robot import IDifferentialRobot


class SimulatedDifferentialRobot(IDifferentialRobot):
    """
    Implementación simulada completa de robot diferencial.
    
    Incluye toda la funcionalidad común más la simulación cinemática.
    """
    
    def __init__(self, wheel_radius: float = 0.025, wheel_base: float = 0.053):
        """
        Inicializa el robot diferencial simulado.
        
        Args:
            wheel_radius: Radio de las ruedas en metros
            wheel_base: Distancia entre ruedas en metros
        """
        self.wheel_radius = wheel_radius
        self.wheel_base = wheel_base
        
        # Estado del robot
        self.pose = RobotPose(0.0, 0.0, 0.0)
        self.left_speed = 0.0   # Velocidad angular rueda izquierda (rad/s)
        self.right_speed = 0.0  # Velocidad angular rueda derecha (rad/s)
    
    def set_motor_speeds(self, left_speed: float, right_speed: float) -> None:
        """Establece las velocidades de los motores."""
        self.left_speed = left_speed
        self.right_speed = right_speed
    
    def get_motor_speeds(self) -> Tuple[float, float]:
        """Obtiene las velocidades actuales de los motores."""
        return (self.left_speed, self.right_speed)
    
    def get_pose(self) -> RobotPose:
        """Obtiene la pose actual del robot."""
        return self.pose.copy()
    
    def set_pose(self, x: float, y: float, theta: float) -> None:
        """Establece la pose del robot."""
        self.pose = RobotPose(x, y, theta)
    
    def stop(self) -> None:
        """Detiene el robot."""
        self.left_speed = 0.0
        self.right_speed = 0.0
    
    def cleanup(self) -> None:
        """Limpieza del robot."""
        self.stop()
    
    # Implementación de la interfaz Ackermann (conversión desde diferencial)
    def set_drive_speed(self, speed: float) -> None:
        """
        Establece la velocidad de avance (interfaz Ackermann).
        Convierte a velocidades diferenciales manteniendo dirección recta.
        
        Args:
            speed: Velocidad lineal en m/s
        """
        # Convertir velocidad lineal a velocidad angular de ruedas
        angular_speed = speed / self.wheel_radius
        self.left_speed = angular_speed
        self.right_speed = angular_speed
    
    def set_steering_angle(self, angle: float) -> None:
        """
        Establece el ángulo de dirección (interfaz Ackermann).
        Convierte a diferencia de velocidades entre ruedas.
        
        Args:
            angle: Ángulo de dirección en radianes (positivo = derecha)
        """
        # Convertir ángulo de dirección a diferencia de velocidades
        # Usando un factor de escala para mapear ángulo a diferencia de velocidades
        base_speed = (self.left_speed + self.right_speed) / 2.0
        speed_diff = angle * base_speed * 2.0  # Factor ajustable
        
        # Aplicar diferencia manteniendo velocidad promedio
        self.left_speed = base_speed - speed_diff / 2.0
        self.right_speed = base_speed + speed_diff / 2.0
    
    def get_drive_speed(self) -> float:
        """
        Obtiene la velocidad de avance actual (interfaz Ackermann).
        
        Returns:
            Velocidad lineal promedio en m/s
        """
        avg_angular_speed = (self.left_speed + self.right_speed) / 2.0
        return avg_angular_speed * self.wheel_radius
    
    def get_steering_angle(self) -> float:
        """
        Obtiene el ángulo de dirección actual (interfaz Ackermann).
        
        Returns:
            Ángulo de dirección estimado en radianes
        """
        # Estimar ángulo basado en diferencia de velocidades
        speed_diff = self.right_speed - self.left_speed
        base_speed = (self.left_speed + self.right_speed) / 2.0
        
        if abs(base_speed) > 0.001:  # Evitar división por cero
            return speed_diff / (base_speed * 2.0)
        return 0.0
    
    def step(self, dt: float) -> None:
        """
        Implementación simulada del paso de tiempo.
        
        Usa el modelo cinemático diferencial para actualizar la pose.
        
        Args:
            dt: Paso de tiempo en segundos
        """
        # Convertir velocidades angulares a velocidades lineales
        v_left = self.left_speed * self.wheel_radius
        v_right = self.right_speed * self.wheel_radius
        
        # Calcular velocidades del robot
        v_linear = (v_left + v_right) / 2
        v_angular = (v_right - v_left) / self.wheel_base
        
        # Obtener orientación actual
        current_theta = self.pose.theta
        
        # Calcular cambios de posición
        dx = v_linear * math.cos(current_theta) * dt
        dy = v_linear * math.sin(current_theta) * dt
        dtheta = v_angular * dt
        
        # Actualizar pose
        self.pose.update(dx, dy, dtheta)
    
    def __str__(self) -> str:
        """Representación en string del robot."""
        return f"SimulatedDifferentialRobot(pose={self.pose}, left={self.left_speed:.2f}, right={self.right_speed:.2f})"
