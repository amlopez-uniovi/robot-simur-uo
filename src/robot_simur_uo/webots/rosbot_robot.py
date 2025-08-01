# Archivo que contiene la clase RosBot para encapsular la configuración del robot
#Precisa un husarion/rosbot estándar con un compass y un gps definidos en el proto
#Se incluye el proto al final

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
from ..interfaces.idifferential_robot import IDifferentialRobot

# Constantes
TIME_STEP = 32
MAX_VELOCITY = 26


class RosBot(WebotsBaseDifferentialRobot):
    """Clase para encapsular la configuración y control del robot RosBot"""
    
    def __init__(self, time_step=TIME_STEP):
        """Inicializar el robot RosBot y sus componentes específicos"""
        # Parámetros físicos del RosBot (extraidos del PROTO)
        wheel_radius = 0.043  # metros
        wheel_base = 0.22      # metros
        
        # Inicializar primero la interfaz diferencial con parámetros
        IDifferentialRobot.__init__(self, wheel_radius, wheel_base)
        
        super().__init__(time_step)
    
    def _init_specific_components(self):
        """Inicializar componentes específicos del RosBot"""
        self._init_motors()
        self._init_position_sensors()
        self._init_cameras()
        self._init_imu()
        self._init_distance_sensors()
        
    def _init_motors(self):
        """Inicializar y configurar los motores del robot"""
        # Obtener dispositivos de motores
        self.front_left_motor = self.robot.getDevice("fl_wheel_joint")
        self.front_right_motor = self.robot.getDevice("fr_wheel_joint")
        self.rear_left_motor = self.robot.getDevice("rl_wheel_joint")
        self.rear_right_motor = self.robot.getDevice("rr_wheel_joint")
        
        # Configurar motores para control de velocidad
        motors = [self.front_left_motor, self.front_right_motor, 
                 self.rear_left_motor, self.rear_right_motor]
        
        for motor in motors:
            motor.setPosition(float('inf'))
            motor.setVelocity(0.0)
    
    def _init_position_sensors(self):
        """Inicializar sensores de posición de las ruedas"""
        self.front_left_position_sensor = self.robot.getDevice("front left wheel motor sensor")
        self.front_right_position_sensor = self.robot.getDevice("front right wheel motor sensor")
        self.rear_left_position_sensor = self.robot.getDevice("rear left wheel motor sensor")
        self.rear_right_position_sensor = self.robot.getDevice("rear right wheel motor sensor")
        
        # Habilitar sensores
        position_sensors = [self.front_left_position_sensor, self.front_right_position_sensor,
                           self.rear_left_position_sensor, self.rear_right_position_sensor]
        
        for sensor in position_sensors:
            sensor.enable(self.time_step)
    
    def _init_cameras(self):
        """Inicializar cámaras RGB y de profundidad"""
        self.camera_rgb = self.robot.getDevice("camera rgb")
        self.camera_depth = self.robot.getDevice("camera depth")
        
        self.camera_rgb.enable(self.time_step)
        self.camera_depth.enable(self.time_step)
    
    def _init_imu(self):
        """Inicializar IMU (acelerómetro, giroscopio y brújula)"""
        self.accelerometer = self.robot.getDevice("imu accelerometer")
        self.gyro = self.robot.getDevice("imu gyro")
        self.imu_compass = self.robot.getDevice("imu compass")
        
        self.accelerometer.enable(self.time_step)
        self.gyro.enable(self.time_step)
        self.imu_compass.enable(self.time_step)
    
    def _init_distance_sensors(self):
        """Inicializar sensores de distancia específicos del RosBot"""
        self.distance_sensors = []
        sensor_names = ["fl_range", "rl_range", "fr_range", "rr_range"]
                
        for name in sensor_names:
            sensor = self.robot.getDevice(name)
            sensor.enable(self.time_step)
            self.distance_sensors.append(sensor)
            
        self.distance_sensors_value = [0] * 4
    
    def get_distance_sensor_values(self):
        """Obtener valores de los sensores de distancia (específico del RosBot)"""
        for i in range(4):
            self.distance_sensors_value[i] = self.distance_sensors[i].getValue()
        return self.distance_sensors_value
    
    def set_motor_velocities(self, left_velocity, right_velocity):
        """Establecer velocidades de los motores (izquierdo y derecho)"""
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
        
        # Aplicar velocidades a los motores físicos (4 ruedas)
        self.front_left_motor.setVelocity(left_velocity)
        self.rear_left_motor.setVelocity(left_velocity)
        self.front_right_motor.setVelocity(right_velocity)
        self.rear_right_motor.setVelocity(right_velocity)
    
    # Implementación de métodos de movimiento para robot diferencial 4x4
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
        # Detener motores físicos directamente (4 ruedas)
        self.front_left_motor.setVelocity(0.0)
        self.rear_left_motor.setVelocity(0.0)
        self.front_right_motor.setVelocity(0.0)
        self.rear_right_motor.setVelocity(0.0)
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
        Sobrescribe para usar motores físicos de Webots con límites específicos del RosBot.
        """
        # Aplicar límites específicos del RosBot (más potente que EPuck)
        max_linear_speed = 0.5  # m/s - Velocidad máxima lineal para RosBot
        max_angular_speed = 1.0  # rad/s - Velocidad angular máxima para RosBot
        
        # Limitar velocidades según capacidades del RosBot
        limited_forward_speed = max(-max_linear_speed, min(max_linear_speed, forward_speed))
        limited_steering_speed = max(-max_angular_speed, min(max_angular_speed, steering_speed))
        
        # Llamar al método padre para conversión con velocidades limitadas
        super().set_drive_command(limited_forward_speed, limited_steering_speed)
        # Aplicar velocidades convertidas usando set_motor_velocities para escalado
        self.set_motor_velocities(self.left_speed, self.right_speed)
    
    def set_forward_speed(self, speed: float) -> None:
        """
        Establece la velocidad de avance (interfaz unificada).
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
    
    # Métodos específicos del RosBot
    def get_accelerometer_values(self):
        """Obtener valores del acelerómetro"""
        return self.accelerometer.getValues()
    
    def get_gyro_values(self):
        """Obtener valores del giroscopio"""
        return self.gyro.getValues()
    
    def get_imu_compass_values(self):
        """Obtener valores de la brújula del IMU"""
        return self.imu_compass.getValues()
    
    def get_camera_rgb_image(self):
        """Obtener imagen RGB de la cámara"""
        return self.camera_rgb.getImage()
    
    def get_camera_depth_image(self):
        """Obtener imagen de profundidad de la cámara"""
        return self.camera_depth.getRangeImage()
    
    def get_position_sensor_values(self):
        """Obtener valores de los sensores de posición de las ruedas"""
        return [
            self.front_left_position_sensor.getValue(),
            self.front_right_position_sensor.getValue(),
            self.rear_left_position_sensor.getValue(),
            self.rear_right_position_sensor.getValue()
        ]
    
    def get_motor_speeds(self) -> Tuple[float, float]:
        """
        Obtiene las velocidades actuales de los motores.
        
        Returns:
            Tupla (velocidad_izquierda, velocidad_derecha) en rad/s
        """
        return (self.left_speed, self.right_speed)

