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

from .webots_base_differential_robot import WebotsDifferentialRobotLGC
from ..interfaces.idifferential_robot import IDifferentialRobot
from ..utils.imu_manager import ImuManager


class RosBot(WebotsDifferentialRobotLGC):
    """
    Clase para encapsular la configuración y control del robot RosBot.

    Ejemplo:
        >>> robot = RosBot()
        >>> # Métodos de sensores y motores disponibles
    """
    
    def __init__(self, time_step=64):
        """Inicializar el robot RosBot y sus componentes específicos"""
        # Parámetros físicos del RosBot (extraidos del PROTO)
        wheel_radius = 0.043  # metros
        wheel_base = 0.22      # metros
        
        # Inicializar primero la interfaz diferencial con parámetros
        super().__init__(wheel_radius, wheel_base, time_step)
        
        self.MAX_VELOCITY = 26*0.99
         
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
        """Inicializar IMU usando ImuManager"""
        self.imu_manager = ImuManager(self.robot, time_step=self.time_step)
    
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

    def set_differential_motor_velocities(self, left_velocity, right_velocity):
        """Establecer velocidades de los motores (izquierdo y derecho)"""
        # Calcular la velocidad máxima solicitada
        max_requested = max(abs(left_velocity), abs(right_velocity))
        
        # Si alguna velocidad excede el límite, escalar proporcionalmente
        if max_requested > self.MAX_VELOCITY:
            scale_ratio = self.MAX_VELOCITY / max_requested
            left_velocity = left_velocity * scale_ratio
            right_velocity = right_velocity * scale_ratio
                
        super().set_differential_motor_velocities(left_velocity, right_velocity)  # Llama a la implementación base
        # Aplicar velocidades a los motores físicos (4 ruedas)
        self.front_left_motor.setVelocity(left_velocity)
        self.rear_left_motor.setVelocity(left_velocity)
        self.front_right_motor.setVelocity(right_velocity)
        self.rear_right_motor.setVelocity(right_velocity)
    
    # El método step se hereda de WebotsDifferentialRobotLGC
    
    
    # Métodos específicos del RosBot
    def get_accelerometer_values(self):
        """Obtener valores del acelerómetro"""
        return self.imu_manager.get_accelerometer()

    def get_gyro_values(self):
        """Obtener valores del giroscopio"""
        return self.imu_manager.get_gyro()

    def get_imu_compass_values(self):
        """Obtener valores de la brújula del IMU"""
        return self.imu_manager.get_compass()
    
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
    
    def log_devices(self, to_terminal: bool = True, to_file: str = None) -> None:
        """
        Log de dispositivos del RosBot - solo datos directos de sensores.
        
        Args:
            to_terminal: Si True, imprime a la terminal
            to_file: Si se especifica, escribe al archivo indicado
        """
        import time
        
        # Generar timestamp
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
        
        # Recopilar información de todos los dispositivos
        log_lines = []
        log_lines.append(f"=== RosBot Device Log - {timestamp} ===")
        
        # 1. GPS Position (datos directos)
        try:
            gps_pos = self.get_gps_position()
            log_lines.append(f"GPS: x={gps_pos[0]:.4f}m, y={gps_pos[1]:.4f}m, z={gps_pos[2]:.4f}m")
        except Exception as e:
            log_lines.append(f"GPS: Error - {e}")
        
        # 2. Compass orientation (datos directos)
        try:
            compass_direction, compass_angle = self.get_compass_orientation()
            log_lines.append(f"Compass: angle={compass_angle:.4f}rad")
            log_lines.append(f"  Direction vector: x={compass_direction[0]:.4f}, y={compass_direction[1]:.4f}, z={compass_direction[2]:.4f}")
        except Exception as e:
            log_lines.append(f"Compass: Error - {e}")
        
        # 3. Motors (velocidades directas - 4 ruedas)
        try:
            log_lines.append(f"Motors (4WD):")
            log_lines.append(f"  Front Left motor velocity: {self.front_left_motor.getVelocity():.4f}rad/s")
            log_lines.append(f"  Rear Left motor velocity: {self.rear_left_motor.getVelocity():.4f}rad/s")
            log_lines.append(f"  Front Right motor velocity: {self.front_right_motor.getVelocity():.4f}rad/s")
            log_lines.append(f"  Rear Right motor velocity: {self.rear_right_motor.getVelocity():.4f}rad/s")
            
            # Posición de motores si está disponible
            try:
                fl_pos = self.front_left_motor.getTargetPosition()
                fr_pos = self.front_right_motor.getTargetPosition()
                rl_pos = self.rear_left_motor.getTargetPosition()
                rr_pos = self.rear_right_motor.getTargetPosition()
                log_lines.append(f"  Motor positions: FL={fl_pos:.4f}rad, FR={fr_pos:.4f}rad")
                log_lines.append(f"                   RL={rl_pos:.4f}rad, RR={rr_pos:.4f}rad")
            except:
                log_lines.append(f"  Motor positions: Not available (velocity mode)")
        except Exception as e:
            log_lines.append(f"Motors: Error - {e}")
        
        # 4. Distance sensors (valores directos - 4 sensores)
        try:
            distance_values = self.get_distance_sensor_values()
            sensor_names = ["fl_range", "rl_range", "fr_range", "rr_range"]
            log_lines.append("Distance Sensors:")
            for i, (name, value) in enumerate(zip(sensor_names, distance_values)):
                log_lines.append(f"  {name}: {value:.4f}m")
        except Exception as e:
            log_lines.append(f"Distance Sensors: Error - {e}")
        
        # 6. IMU Accelerometer (valores directos)
        try:
            accel_values = self.get_accelerometer_values()
            log_lines.append(f"IMU Accelerometer: x={accel_values[0]:.4f}, y={accel_values[1]:.4f}, z={accel_values[2]:.4f} m/s²")
        except Exception as e:
            log_lines.append(f"IMU Accelerometer: Error - {e}")
        
        # 7. IMU Gyroscope (valores directos)
        try:
            gyro_values = self.get_gyro_values()
            log_lines.append(f"IMU Gyroscope: x={gyro_values[0]:.4f}, y={gyro_values[1]:.4f}, z={gyro_values[2]:.4f} rad/s")
        except Exception as e:
            log_lines.append(f"IMU Gyroscope: Error - {e}")
        
        # 8. IMU Compass (valores directos)
        try:
            imu_compass_values = self.get_imu_compass_values()
            log_lines.append(f"IMU Compass: x={imu_compass_values[0]:.4f}, y={imu_compass_values[1]:.4f}, z={imu_compass_values[2]:.4f}")
        except Exception as e:
            log_lines.append(f"IMU Compass: Error - {e}")
        
        # 9. Position Sensors (encoders de las ruedas)
        try:
            position_values = self.get_position_sensor_values()
            log_lines.append("Wheel Position Sensors:")
            log_lines.append(f"  Front Left: {position_values[0]:.4f}rad")
            log_lines.append(f"  Front Right: {position_values[1]:.4f}rad")
            log_lines.append(f"  Rear Left: {position_values[2]:.4f}rad")
            log_lines.append(f"  Rear Right: {position_values[3]:.4f}rad")
        except Exception as e:
            log_lines.append(f"Position Sensors: Error - {e}")
        
        # 10. Cámaras RGB y Depth (propiedades directas)
        try:
            log_lines.append(f"Cameras:")
            # Cámara RGB
            log_lines.append(f"  RGB Camera:")
            log_lines.append(f"    Width: {self.camera_rgb.getWidth()}px")
            log_lines.append(f"    Height: {self.camera_rgb.getHeight()}px")
            log_lines.append(f"    FOV: {self.camera_rgb.getFov():.4f}rad")
            log_lines.append(f"    Near: {self.camera_rgb.getNear():.4f}m")
            log_lines.append(f"    Far: {self.camera_rgb.getFar():.4f}m")
            
            # Cámara de profundidad
            log_lines.append(f"  Depth Camera:")
            log_lines.append(f"    Width: {self.camera_depth.getWidth()}px")
            log_lines.append(f"    Height: {self.camera_depth.getHeight()}px")
            log_lines.append(f"    Min range: {self.camera_depth.getMinRange():.4f}m")
            log_lines.append(f"    Max range: {self.camera_depth.getMaxRange():.4f}m")
            
            # Estado de imágenes
            try:
                rgb_image = self.camera_rgb.getImage()
                depth_image = self.camera_depth.getRangeImage()
                if rgb_image:
                    log_lines.append(f"    RGB image data: {len(rgb_image)} bytes available")
                else:
                    log_lines.append(f"    RGB image data: Not available")
                if depth_image:
                    log_lines.append(f"    Depth image data: {len(depth_image)} values available")
                else:
                    log_lines.append(f"    Depth image data: Not available")
            except:
                log_lines.append(f"    Image data: Access error")
        except Exception as e:
            log_lines.append(f"Cameras: Error - {e}")
        
        # 11. Parámetros del robot (configuración)
        log_lines.append(f"Robot Configuration:")
        log_lines.append(f"  Time step: {self.time_step}ms")
        log_lines.append(f"  Wheel radius: {self.wheel_radius:.4f}m")
        log_lines.append(f"  Wheel base: {self.wheel_base:.4f}m")
        log_lines.append(f"  Max velocity: {self.MAX_VELOCITY:.4f}rad/s")
        log_lines.append(f"  Drive type: 4WD (Four Wheel Drive)")
        
        log_lines.append("=" * 70)
        
        # Almacenar el mensaje en el atributo de la clase
        self.log_message = "\n".join(log_lines)
        
        # Llamar al método base para manejar la salida
        super().log_devices(to_terminal, to_file)


