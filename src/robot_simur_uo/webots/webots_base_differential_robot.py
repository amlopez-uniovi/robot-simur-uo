try:
    from controller import Robot
except ImportError:
    # Si no se encuentra el módulo `controller`, define un stub o lanza una advertencia
    class Robot:
        def __init__(self, *args, **kwargs):
            raise RuntimeError("El módulo `controller` solo está disponible en el entorno de Webots.")

import math
from typing import Tuple
from ..interfaces.idifferential_robot import IDifferentialRobot
from ..utils.coordinates import RobotPose
from ..utils.lidar_manager import LidarManager
from ..utils.gps_manager import GpsManager
from ..utils.compass_manager import CompassManager

class WebotsBaseDifferentialRobot(IDifferentialRobot):
    """
    Clase base para robots diferenciales en Webots con funcionalidades comunes.

    Ejemplo:
        >>> robot = WebotsBaseDifferentialRobot()
        >>> # Métodos de inicialización y sensores disponibles
    """
    
    def __init__(self, time_step=64):
        """Inicializar el robot base y sus componentes comunes"""
        self.robot = Robot()
        self.time_step = time_step
        
        # Inicializar componentes comunes
        self._init_common_components()
        
        # Inicializar componentes específicos del robot (implementado en subclases)
        self._init_specific_components()
    
    def _init_common_components(self):
        """Inicializar componentes comunes a todos los robots"""
        self.gps_manager = GpsManager(self.robot, time_step=self.time_step)
        self.compass_manager = CompassManager(self.robot, time_step=self.time_step)
        # Inicializar LidarManager
        self.lidar_manager = None
        self._init_lidar_manager()
        

    
    def _init_specific_components(self):
        """Inicializar componentes específicos del robot (debe ser implementado por subclases)"""
        raise NotImplementedError("Subclases deben implementar _init_specific_components")
    
    # Eliminado: _init_navigation_sensors (ahora gestionado por managers)
    
    def _init_lidar_manager(self):
        """Inicializar LidarManager con auto-detección del dispositivo"""
        try:
            # Lista de nombres posibles para dispositivos LiDAR
            possible_names = ["lidar", "laser", "Lidar", "LIDAR"]
            device_name = None
            
            # Buscar el primer dispositivo disponible
            for name in possible_names:
                try:
                    test_device = self.robot.getDevice(name)
                    if test_device is not None:
                        device_name = name
                        print(f"🔍 Dispositivo LiDAR encontrado: '{name}'")
                        break
                except:
                    continue
            
            if device_name:
                self.lidar_manager = LidarManager(
                    robot=self.robot, 
                    device_name=device_name, 
                    time_step=self.time_step
                )
                print(f"✅ LidarManager inicializado con dispositivo '{device_name}'")
            else:
                print("⚠️ No se encontró ningún dispositivo LiDAR en el robot")
                self.lidar_manager = None
                
        except Exception as e:
            print(f"⚠️ Error inicializando LidarManager: {e}")
            self.lidar_manager = None
    
    def step(self, time_step=None):
        """Ejecutar un paso de simulación
        
        Args:
            time_step (int, optional): Duración del paso en milisegundos. 
                                      Si no se especifica, usa self.time_step
        
        Returns:
            int: 0 si la simulación continúa, -1 si debe terminar
        """
        if time_step is None:
            time_step = self.time_step
        return self.robot.step(time_step)
    
    def get_gps_position(self):
        """Obtener posición GPS usando GpsManager"""
        return self.gps_manager.get_position()

    def get_compass_orientation(self):
        """Obtener orientación de la brújula usando CompassManager"""
        direction = self.compass_manager.get_direction()
        angle = math.pi / 2 - math.atan2(direction[1], direction[0])
        return direction, angle
    
    def get_lidar_data(self):
        """Obtener datos del lidar/laser"""
        if self.has_lidar_manager():
            return self.lidar_manager.get_raw_data()
        return []
    
    
    def get_lidar_manager(self):
        """Obtener el LidarManager configurado"""
        return self.lidar_manager
    
    def has_lidar_manager(self):
        """Verificar si el LidarManager está disponible"""
        return self.lidar_manager is not None and self.lidar_manager.is_available()
    
    
    def cleanup(self):
        """Limpiar recursos al finalizar"""
        # Llamar al método de la interfaz base que detiene el robot
        super().cleanup()
    
    # Implementación de la interfaz IRobot
    def set_motor_speeds(self, left_speed: float, right_speed: float) -> None:
        """
        Establece las velocidades de los motores.
        
        Args:
            left_speed: Velocidad del motor izquierdo (rad/s)
            right_speed: Velocidad del motor derecho (rad/s)
        """
        # Debe ser implementado por subclases específicas
        raise NotImplementedError("Subclases deben implementar set_motor_speeds")
    
    def get_pose(self) -> RobotPose:
        """
        Obtiene la pose actual del robot.
        
        Returns:
            Pose actual del robot
        """
        gps_position = self.get_gps_position()
        compass_direction, angle = self.get_compass_orientation()
        
        return RobotPose(gps_position[0], gps_position[1], angle)
    
    def set_pose(self, x: float, y: float, theta: float) -> None:
        """
        Establece la pose del robot.
        
        Nota: En Webots esto normalmente no es posible durante la simulación.
        Este método existe para compatibilidad con la interfaz.
        """
        print(f"Advertencia: set_pose no está soportado en robots de Webots durante la simulación")
    
    def get_motor_speeds(self) -> Tuple[float, float]:
        """
        Obtiene las velocidades actuales de los motores.
        
        Returns:
            Tupla (velocidad_izquierda, velocidad_derecha) en rad/s
        """
        # Debe ser implementado por subclases específicas
        raise NotImplementedError("Subclases deben implementar get_motor_speeds")
    
    def log_lidar_data(self) -> list:
        """
        Genera el logging de datos del LiDAR utilizando LidarManager.
        Método común que pueden usar todas las subclases.
        
        Returns:
            list: Lista de strings con la información del LiDAR para logging
        """
        log_lines = []
        
        try:
            # Verificar si el LiDAR está disponible
            if not self.has_lidar_manager():
                log_lines.append("Lidar: No disponible")
                return log_lines
                
            # Usar LidarManager para obtener información del LiDAR
            lidar_manager = self.get_lidar_manager()
            
            # Agregar el resumen del LiDAR usando las funciones del LidarManager
            lidar_summary = lidar_manager.print_summary()
            if lidar_summary:
                log_lines.extend(lidar_summary.split('\n'))
            else:
                log_lines.append("Lidar: Sin datos disponibles")
                
        except Exception as e:
            log_lines.append(f"Lidar: Error - {e}")
            
        return log_lines
    
    def log_devices(self, to_terminal: bool = True, to_file: str = None) -> None:
        """
        Método base para logging de dispositivos.
        Añade automáticamente información del LiDAR y maneja la salida a terminal y archivo.
        
        Args:
            to_terminal: Si True, imprime a la terminal
            to_file: Si se especifica, escribe al archivo indicado
        """
        # Las subclases deben establecer self.log_message antes de llamar a este método
        if hasattr(self, 'log_message') and self.log_message:
            # Convertir el mensaje a lista de líneas para poder agregar LiDAR
            log_lines = self.log_message.split('\n')
            
            # Buscar dónde insertar la información del LiDAR (antes de la línea de separación final)
            insert_index = len(log_lines) - 1  # Por defecto al final
            for i, line in enumerate(log_lines):
                if line.startswith("=" * 10):  # Buscar línea de separación final
                    insert_index = i
                    break
            
            # Agregar información del LiDAR
            try:
                lidar_log_lines = self.log_lidar_data()
                # Insertar las líneas del LiDAR antes de la separación final
                for j, lidar_line in enumerate(lidar_log_lines):
                    log_lines.insert(insert_index + j, lidar_line)
            except Exception as e:
                log_lines.insert(insert_index, f"Lidar: Error - {e}")
            
            # Reconstruir el mensaje final
            final_message = '\n'.join(log_lines)
            
            # Salida a terminal
            if to_terminal:
                print(final_message)
            
            # Salida a archivo
            if to_file:
                try:
                    with open(to_file, 'a', encoding='utf-8') as f:
                        f.write(f"{final_message}\n\n")
                except Exception as e:
                    print(f"Error escribiendo a archivo {to_file}: {e}")
        else:
            print("Warning: No log message available. Subclases should set self.log_message before calling super().log_devices()")
    
    # Métodos abstractos que deben ser implementados por subclases
    def _init_motors(self):
        """Inicializar motores específicos del robot (debe ser implementado por subclases)"""
        raise NotImplementedError("Subclases deben implementar _init_motors")
    
    def move_forward(self, speed=2.0):
        """Mover el robot hacia adelante (debe ser implementado por subclases)"""
        raise NotImplementedError("Subclases deben implementar move_forward")
    
    def move_backward(self, speed=2.0):
        """Mover el robot hacia atrás (debe ser implementado por subclases)"""
        raise NotImplementedError("Subclases deben implementar move_backward")
    
    def turn_left(self, speed=2.0):
        """Girar el robot a la izquierda (debe ser implementado por subclases)"""
        raise NotImplementedError("Subclases deben implementar turn_left")
    
    def turn_right(self, speed=2.0):
        """Girar el robot a la derecha (debe ser implementado por subclases)"""
        raise NotImplementedError("Subclases deben implementar turn_right")
