# Archivo que contiene la clase EPuck para encapsular la configuración del robot e-puck
# Precisa un e-puck estándar con sensores de distancia, GPS y brújula definidos en el proto

# Importar las librerías de Webots
import math
import sys
import os
from typing import Tuple

try:
    from controller import Robot
except ImportError:
    # Si no se encuentra el módulo `controller`, define un stub o lanza una advertencia
    class Robot:
        def __init__(self, *args, **kwargs):
            raise RuntimeError("El módulo `controller` solo está disponible en el entorno de Webots.")


# Importar WebotsBaseDifferentialRobot del mismo paquete
from .webots_base_differential_robot import WebotsBaseDifferentialRobot

# Constantes
TIME_STEP = 64
MAX_VELOCITY = 6.28  # Velocidad máxima típica para e-puck

class EPuck(WebotsBaseDifferentialRobot):
    """Clase para encapsular la configuración y control del robot e-puck"""
    
    def __init__(self, time_step=TIME_STEP):
        """Inicializar el robot e-puck y sus componentes específicos"""
        # Nombres de los sensores de distancia del e-puck
        self.distance_sensors_names = ["ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"]
        
        # Parámetros físicos del e-puck
        wheel_radius = 0.0205  # metros
        wheel_base = 0.052     # metros
        
        # Inicializar primero la interfaz diferencial con parámetros
        from ..interfaces.idifferential_robot import IDifferentialRobot
        IDifferentialRobot.__init__(self, wheel_radius, wheel_base)
            
        super().__init__(time_step)
    
    def _init_specific_components(self):
        """Inicializar componentes específicos del e-puck"""
        self._init_motors()
        self._init_distance_sensors()
    
    def _init_motors(self):
        """Inicializar y configurar los motores del robot"""
        # Obtener dispositivos de motores (solo 2 motores en e-puck)
        self.left_motor = self.robot.getDevice("left wheel motor")
        self.right_motor = self.robot.getDevice("right wheel motor")
        
        # Configurar motores para control de velocidad
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
    
    def _init_distance_sensors(self):
        """Inicializar sensores de distancia específicos del e-puck"""
        self.distance_sensors = []
        
        for name in self.distance_sensors_names:
            sensor = self.robot.getDevice(name)
            sensor.enable(self.time_step)
            self.distance_sensors.append(sensor)
            
        self.distance_sensors_value = [0] * 8
    
    def get_distance_sensor_values(self):
        """Obtener valores de los sensores de distancia (específico del e-puck)"""
        for i in range(8):
            self.distance_sensors_value[i] = self.distance_sensors[i].getValue()
        return self.distance_sensors_value
    
    def get_distance_sensor_value(self, index):
        """Obtener valor de un sensor de distancia específico
        
        Args:
            index (int): Índice del sensor (0-7)
        
        Returns:
            float: Valor del sensor de distancia
        """
        if 0 <= index < 8:
            return self.distance_sensors[index].getValue()
        else:
            raise IndexError(f"Índice de sensor {index} fuera de rango (0-7)")
    
    def set_motor_velocities(self, left_velocity, right_velocity):
        """Establecer velocidades de los motores (izquierdo y derecho)
        
        Args:
            left_velocity (float): Velocidad del motor izquierdo
            right_velocity (float): Velocidad del motor derecho
        """
        # Calcular la velocidad máxima solicitada
        max_requested = max(abs(left_velocity), abs(right_velocity))
        
        # Si alguna velocidad excede el límite, escalar proporcionalmente
        if max_requested > MAX_VELOCITY:
            scale_ratio = MAX_VELOCITY / max_requested
            left_velocity = left_velocity * scale_ratio
            right_velocity = right_velocity * scale_ratio
        
        # Actualizar atributos de la interfaz diferencial
        self.left_speed = left_velocity
        self.right_speed = right_velocity
        
        # Aplicar velocidades a los motores físicos
        self.left_motor.setVelocity(left_velocity)
        self.right_motor.setVelocity(right_velocity)
    
    # Implementación de métodos de movimiento para robot diferencial
    def move_forward(self, speed=2.0):
        """Mover el robot hacia adelante"""
        self.set_motor_velocities(speed, speed)
    
    def move_backward(self, speed=2.0):
        """Mover el robot hacia atrás"""
        self.set_motor_velocities(-speed, -speed)
    
    def turn_left(self, speed=2.0):
        """Girar el robot a la izquierda"""
        self.set_motor_velocities(-speed, speed)
    
    def turn_right(self, speed=2.0):
        """Girar el robot a la derecha"""
        self.set_motor_velocities(speed, -speed)
    
    def stop(self):
        """Detener el robot"""
        # Detener motores físicos directamente
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
        # Actualizar atributos de la interfaz
        self.left_speed = 0.0
        self.right_speed = 0.0
    
    def step(self, dt: float = None) -> int:
        """
        Ejecuta un paso de simulación de Webots.
        
        Args:
            dt: Paso de tiempo (ignorado en Webots, usa time_step interno)
            
        Returns:
            int: -1 si la simulación termina, 0 en caso contrario
        """
        return self.robot.step(self.time_step)
    
    # Sobrescribir métodos de interfaz para usar motores físicos
    def set_drive_command(self, forward_speed: float, steering_speed: float) -> None:
        """
        Establece velocidad y dirección simultáneamente (interfaz principal).
        Sobrescribe para usar motores físicos de Webots con límites específicos del EPuck.
        """
        # Aplicar límites específicos del EPuck
        max_linear_speed = 0.1  # m/s - Velocidad máxima lineal conservadora para EPuck
        max_angular_speed = 0.5  # rad/s - Velocidad angular máxima conservadora para EPuck
        
        # Limitar velocidades según capacidades del EPuck
        limited_forward_speed = max(-max_linear_speed, min(max_linear_speed, forward_speed))
        limited_steering_speed = max(-max_angular_speed, min(max_angular_speed, steering_speed))
        
        # Llamar al método padre para conversión con velocidades limitadas
        super().set_drive_command(limited_forward_speed, limited_steering_speed)
        # Aplicar velocidades convertidas usando set_motor_velocities para escalado
        self.set_motor_velocities(self.left_speed, self.right_speed)
    
    def set_forward_speed(self, speed: float) -> None:
        """
        Establece la velocidad de avance.
        Sobrescribe para usar motores físicos de Webots.
        """
        # Llamar al método padre para mantener conversión
        super().set_forward_speed(speed)
        # Aplicar velocidades convertidas usando set_motor_velocities para escalado
        self.set_motor_velocities(self.left_speed, self.right_speed)
    
    def set_steering_speed(self, speed: float) -> None:
        """
        Establece la velocidad de dirección.
        Sobrescribe para usar motores físicos de Webots.
        """
        # Llamar al método padre para mantener conversión
        super().set_steering_speed(speed)
        # Aplicar velocidades convertidas usando set_motor_velocities para escalado
        self.set_motor_velocities(self.left_speed, self.right_speed)
    
    # Métodos específicos para sensores de distancia del e-puck
    def get_front_sensors_average(self):
        """Obtener el promedio de los sensores frontales (ps0, ps1, ps6, ps7)"""
        front_sensors = [0, 1, 6, 7]  # Índices de los sensores frontales
        values = [self.distance_sensors[i].getValue() for i in front_sensors]
        return sum(values) / len(values)
    
    def get_left_sensors_average(self):
        """Obtener el promedio de los sensores izquierdos (ps5, ps6, ps7)"""
        left_sensors = [5, 6, 7]  # Índices de los sensores izquierdos
        values = [self.distance_sensors[i].getValue() for i in left_sensors]
        return sum(values) / len(values)
    
    def get_right_sensors_average(self):
        """Obtener el promedio de los sensores derechos (ps0, ps1, ps2)"""
        right_sensors = [0, 1, 2]  # Índices de los sensores derechos
        values = [self.distance_sensors[i].getValue() for i in right_sensors]
        return sum(values) / len(values)
    
    def obstacle_detected(self, threshold=80.0):
        """Detectar si hay un obstáculo cerca
        
        Args:
            threshold (float): Umbral de detección
        
        Returns:
            bool: True si hay obstáculo, False si no
        """
        front_value = self.get_front_sensors_average()
        return front_value > threshold
    
    def get_motor_speeds(self) -> Tuple[float, float]:
        """
        Obtiene las velocidades actuales de los motores.
        
        Returns:
            Tupla (velocidad_izquierda, velocidad_derecha) en rad/s
        """
        return (self.left_speed, self.right_speed)
