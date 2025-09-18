from ..utils.rgb_camera_manager import RgbCameraManager
from ..utils.gps_manager import GpsManager
from ..utils.compass_manager import CompassManager
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

from .webots_base_differential_robot import WebotsDifferentialRobotLGC

class EPuck(WebotsDifferentialRobotLGC):
    """
    Clase para encapsular la configuración y control del robot e-puck.

    Ejemplo:
        >>> robot = EPuck()
        >>> pos = robot.get_gps_position()
        >>> print(pos)
    """
    
    def __init__(self, time_step=64):
        # Nombres de los sensores de distancia del e-puck
        self.distance_sensors_names = ["ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"]

        # Parámetros físicos del e-puck
        wheel_radius = 0.0205  # metros
        wheel_base = 0.052     # metros

        # Inicializar la jerarquía correctamente
        super().__init__(wheel_radius, wheel_base, time_step)
        self.MAX_VELOCITY = 6.28*0.99  # Velocidad máxima para motores e-puck
    
    def _init_lidar_manager(self):
        """Inicializar LidarManager con auto-detección del dispositivo"""
        super()._init_lidar_manager()
        
        print("Configurando LidarManager para e-puck...")
        self.lidar_manager.set_sweep_range((math.pi, -math.pi))  # Rango completo de 360 grados
        
    def _init_specific_components(self  ):
        """Inicializar componentes específicos del e-puck"""
        self._init_motors()
        self._init_distance_sensors()
        self.gps_manager = GpsManager(self.robot, time_step=self.time_step)
        self.compass_manager = CompassManager(self.robot, time_step=self.time_step)
        self.rgb_camera_manager = RgbCameraManager(self.robot, device_name="camera", time_step=self.time_step)

    def get_rgb_camera_manager(self):
        """Obtener el manager de la cámara RGB del e-puck"""
        return self.rgb_camera_manager
    
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
        distance_sensors_values = [0] * 8
        for i in range(8):
            distance_sensors_values[i] = self.distance_sensors[i].getValue()
        return distance_sensors_values


    def set_differential_motor_velocities(self, left_velocity, right_velocity):
        """Establecer velocidades de los motores (izquierdo y derecho)
        
        Args:
            left_velocity (float): Velocidad del motor izquierdo
            right_velocity (float): Velocidad del motor derecho
        """
        
        # Calcular la velocidad máxima solicitada
        max_requested = max(abs(left_velocity), abs(right_velocity))
        
        # Si alguna velocidad excede el límite, escalar proporcionalmente
        if max_requested > self.MAX_VELOCITY:
            scale_ratio = self.MAX_VELOCITY / max_requested
            left_velocity = left_velocity * scale_ratio
            right_velocity = right_velocity * scale_ratio
               
        super().set_differential_motor_velocities(left_velocity, right_velocity)  # Llama a la implementación base
        
        # Aplicar velocidades a los motores físicos
        self.left_motor.setVelocity(left_velocity)
        self.right_motor.setVelocity(right_velocity)
    


    def log_devices(self, to_terminal: bool = True, to_file: str = None) -> None:
        """
        Log de dispositivos del EPuck - solo datos directos de sensores.
        
        Args:
            to_terminal: Si True, imprime a la terminal
            to_file: Si se especifica, escribe al archivo indicado
        """
        import time
        
        # Generar timestamp
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
        
        # Recopilar información de todos los dispositivos
        log_lines = []
        log_lines.append(f"=== EPuck Device Log - {timestamp} ===")
        
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
        
        # 3. Motors (velocidades directas)
        try:
            log_lines.append(f"Motors:")
            log_lines.append(f"  Left motor velocity: {self.left_motor.getVelocity():.4f}rad/s")
            log_lines.append(f"  Right motor velocity: {self.right_motor.getVelocity():.4f}rad/s")
            
            # Posición de motores si está disponible
            try:
                left_position = self.left_motor.getTargetPosition()
                right_position = self.right_motor.getTargetPosition()
                log_lines.append(f"  Motor positions: left={left_position:.4f}rad, right={right_position:.4f}rad")
            except:
                log_lines.append(f"  Motor positions: Not available (velocity mode)")
        except Exception as e:
            log_lines.append(f"Motors: Error - {e}")
        
        # 4. Distance sensors (valores directos)
        try:
            distance_values = self.get_distance_sensor_values()
            log_lines.append("Distance Sensors:")
            for i, value in enumerate(distance_values):
                log_lines.append(f"  ps{i}: {value:.2f}")
        except Exception as e:
            log_lines.append(f"Distance Sensors: Error - {e}")
            
        # 6. Cámara (usando manager)
        try:
            camera_manager = self.get_rgb_camera_manager()
            log_lines.append(f"Camera:")
            width, height = camera_manager.get_resolution()
            log_lines.append(f"  Width: {width}px")
            log_lines.append(f"  Height: {height}px")
            # Si el manager expone FOV, Near, Far, puedes añadirlo aquí
            # Estado de imagen
            try:
                image = camera_manager.get_image()
                if image is not None:
                    log_lines.append(f"  Image data: {image.shape} (numpy array)")
                else:
                    log_lines.append(f"  Image data: Not available")
            except Exception as e:
                log_lines.append(f"  Image data: Access error - {e}")
        except Exception as e:
            log_lines.append(f"Camera: Error - {e}")
                
                
        # 9. LEDs (conteo directo)
        try:
            led_count = 0
            for i in range(10):
                try:
                    led = self.robot.getDevice(f"led{i}")
                    if led:
                        led_count += 1
                except:
                    break
            if led_count > 0:
                log_lines.append(f"LEDs: {led_count} devices available")
            else:
                log_lines.append("LEDs: Not available")
        except Exception as e:
            log_lines.append(f"LEDs: Error - {e}")
        
        # 10. Parámetros del robot (configuración)
        log_lines.append(f"Robot Configuration:")
        log_lines.append(f"  Time step: {self.time_step}ms")
        log_lines.append(f"  Wheel radius: {self.wheel_radius:.4f}m")
        log_lines.append(f"  Wheel base: {self.wheel_base:.4f}m")
        log_lines.append(f"  Max velocity: {self.MAX_VELOCITY:.4f}rad/s")
        
        log_lines.append("=" * 70)
        
        # Almacenar el mensaje en el atributo de la clase
        self.log_message = "\n".join(log_lines)
        
        # Llamar al método base para manejar la salida
        super().log_devices(to_terminal, to_file)