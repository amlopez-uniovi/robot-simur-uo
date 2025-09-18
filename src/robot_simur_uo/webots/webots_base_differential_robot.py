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

class WebotsDifferentialRobotLGC(IDifferentialRobot):
    """
    Clase base para robots diferenciales en Webots con sensores LiDAR, GPS y brújula (Compass).

    """
    #Inicializaciones ...
    def __init__(self, wheel_radius: float, wheel_base: float, time_step=64):
        super().__init__(wheel_radius, wheel_base)  # Llama al __init__ de IDifferentialRobot
        self.robot = Robot() #Llamada a la API de Webots
        self.time_step = time_step
        self._init_common_components()
        self._init_specific_components()    
 
    def cleanup(self):
        """Limpiar recursos al finalizar"""
        # Llamar al método de la interfaz base que detiene el robot
        super().cleanup()
       
    def _init_motors(self):
        """Inicializar motores específicos del robot (debe ser implementado por subclases)"""
        raise NotImplementedError("Subclases deben implementar _init_motors")

    def _init_common_components(self):
        """Inicializar componentes comunes a todos los robots que usamos, algunos añadidos por nosotros
        Asumimos que todos tienen GPS, Compass y Lidar
        """
        self._init_gps_manager()
        self._init_compass_manager()
        self._init_lidar_manager()

    def _init_specific_components(self):
        """Inicializar componentes específicos del robot (debe ser implementado por subclases)"""
        raise NotImplementedError("Subclases deben implementar _init_specific_components")

    def _init_gps_manager(self):
        """Inicializar GpsManager con auto-detección del dispositivo"""
        try:
            possible_names = ["gps", "GPS", "Gps"]
            device_name = None
            
            for name in possible_names:
                try:
                    test_device = self.robot.getDevice(name)
                    if test_device is not None:
                        device_name = name
                        print(f"🔍 Dispositivo GPS encontrado: '{name}'")
                        break
                except:
                    continue
            
            if device_name:
                self.gps_manager = GpsManager(self.robot, device_name=device_name, time_step=self.time_step)
                print(f"✅ GpsManager inicializado con dispositivo '{device_name}'")
            else:
                print("⚠️ No se encontró ningún dispositivo GPS en el robot")
                self.gps_manager = None
            
        except Exception as e:
            print(f"⚠️ Error inicializando GpsManager: {e}")
            self.gps_manager = None
            
    def _init_compass_manager(self):
        """Inicializar CompassManager con auto-detección del dispositivo"""
        try:
            possible_names = ["compass", "Compass", "COMPASS"]
            device_name = None
            
            for name in possible_names:
                try:
                    test_device = self.robot.getDevice(name)
                    if test_device is not None:
                        device_name = name
                        print(f"🔍 Dispositivo Compass encontrado: '{name}'")
                        break
                except:
                    continue

            if device_name:
                self.compass_manager = CompassManager(self.robot, device_name=device_name, time_step=self.time_step)
                print(f"✅ CompassManager inicializado con dispositivo '{device_name}'")
            else:
                print("⚠️ No se encontró ningún dispositivo Compass en el robot")
                self.compass_manager = None

        except Exception as e:
            print(f"⚠️ Error inicializando CompassManager: {e}")
            self.compass_manager = None

    def _init_lidar_manager(self):
        """Inicializar LidarManager con auto-detección del dispositivo"""
        try:
            # Lista de nombres posibles para dispositivos LiDAR
            possible_names = ["laser", "lidar", "Lidar", "LIDAR"]
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

    #Acceso a managers de dispositivos
            
    def get_lidar_manager(self):
        """Obtener el LidarManager configurado"""
        return self.lidar_manager
    
    def get_compass_manager(self):
        """Obtener el CompassManager configurado"""
        return self.compass_manager
    
    def get_gps_manager(self):
        """Obtener el GpsManager configurado"""
        return self.gps_manager

    #Gestión de la Pose
                
    def get_pose(self) -> RobotPose:
        """
        Obtiene la pose actual del robot.
        
        Returns:
            Pose actual del robot
        """
        gps_position = self.gps_manager.get_position()

        direction = self.compass_manager.get_direction()
        angle = math.pi / 2 - math.atan2(direction[1], direction[0])

        self.pose = RobotPose(gps_position[0], gps_position[1], angle)

        return self.pose
    
    def set_pose(self, pose: RobotPose) -> None:
        """
        Establece la pose del robot.
        
        Nota: En Webots esto normalmente no es posible durante la simulación.
        Este método existe para compatibilidad con la interfaz.
        """
        print(f"Advertencia: set_pose no está soportado en robots de Webots durante la simulación")
    
    #Funciones de log
    
    def _log_lidar_data(self) -> list:
        """
        Genera el logging de datos del LiDAR utilizando LidarManager.
        Método común que pueden usar todas las subclases.
    
        Returns:
            list: Lista de strings con la información del LiDAR para logging
        """
        log_lines = []
        try:
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
                lidar_log_lines = self._log_lidar_data()
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
    
    # Evolución de la simulación
    def step(self, time_step=None):
        """
        Ejecuta un paso de simulación en Webots de forma uniforme para cualquier robot.

        Args:
            time_step (int, optional): Duración del paso en milisegundos. Si no se especifica, se usa self.time_step.

        Returns:
            int: 0 si la simulación continúa, -1 si debe terminar.

        Ejemplo de uso:
            >>> robot = EPuck()
            >>> while robot.step() != -1:
            ...     # lógica de control
        """
        if time_step is None:
            time_step = self.time_step
        return self.robot.step(time_step)
     