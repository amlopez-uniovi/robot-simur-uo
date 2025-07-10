"""
Configuración global para robots.
"""

from typing import Dict, Any, Optional, List
import json
import os


class RobotConfig:
    """Clase para manejar configuraciones de robots."""
    
    # Configuración por defecto
    DEFAULT_CONFIG = {
        # Parámetros físicos generales
        'robot': {
            'wheel_radius': 0.025,  # metros
            'wheel_base': 0.1,      # metros
            'max_speed': 1.0,       # m/s
            'max_angular_speed': 2.0,  # rad/s
        },
        
        # Configuración de sensores
        'sensors': {
            'lidar': {
                'max_range': 5.0,
                'angular_resolution': 0.01,
                'noise_std': 0.02
            },
            'ultrasonic': {
                'max_range': 2.0,
                'beam_width': 0.2,
                'noise_std': 0.05
            },
            'camera': {
                'width': 640,
                'height': 480,
                'fov': 1.57,  # radianes (90 grados)
                'focal_length': 500.0
            }
        },
        
        # Parámetros de control
        'control': {
            'navigation': {
                'position_tolerance': 0.05,  # metros
                'angle_tolerance': 0.1,      # radianes
                'max_linear_acceleration': 1.0,
                'max_angular_acceleration': 2.0
            },
            'pid': {
                'linear': {'kp': 1.0, 'ki': 0.0, 'kd': 0.1},
                'angular': {'kp': 2.0, 'ki': 0.0, 'kd': 0.2}
            },
            'obstacle_avoidance': {
                'safe_distance': 0.3,
                'reaction_distance': 0.5,
                'avoidance_gain': 1.5
            }
        },
        
        # Configuración de simulación
        'simulation': {
            'time_step': 0.032,      # segundos (Webots default)
            'update_rate': 30,       # Hz
            'enable_physics': True,
            'gravity': 9.81
        },
        
        # Configuración específica por robot
        'robot_types': {
            'epuck': {
                'wheel_radius': 0.0205,
                'wheel_base': 0.053,
                'max_speed': 0.12,
                'sensor_count': 8,
                'sensor_range': 0.05
            },
            'rosbot': {
                'wheel_radius': 0.0425,
                'wheel_base': 0.2,
                'max_speed': 1.0,
                'has_lidar': True,
                'lidar_range': 12.0
            }
        }
    }
    
    def __init__(self, config_file: Optional[str] = None):
        """
        Inicializa la configuración.
        
        Args:
            config_file: Archivo de configuración JSON opcional
        """
        self.config = self.DEFAULT_CONFIG.copy()
        
        if config_file and os.path.exists(config_file):
            self.load_from_file(config_file)
    
    def get(self, key_path: str, default: Any = None) -> Any:
        """
        Obtiene un valor de configuración usando notación de punto.
        
        Args:
            key_path: Ruta de la clave (ej: 'robot.wheel_radius')
            default: Valor por defecto si no existe
            
        Returns:
            Valor de configuración
        """
        keys = key_path.split('.')
        current = self.config
        
        try:
            for key in keys:
                current = current[key]
            return current
        except (KeyError, TypeError):
            return default
    
    def set(self, key_path: str, value: Any):
        """
        Establece un valor de configuración.
        
        Args:
            key_path: Ruta de la clave
            value: Nuevo valor
        """
        keys = key_path.split('.')
        current = self.config
        
        # Navegar hasta el penúltimo nivel
        for key in keys[:-1]:
            if key not in current:
                current[key] = {}
            current = current[key]
        
        # Establecer el valor final
        current[keys[-1]] = value
    
    def update(self, updates: Dict[str, Any]):
        """
        Actualiza múltiples configuraciones.
        
        Args:
            updates: Diccionario con actualizaciones
        """
        for key_path, value in updates.items():
            self.set(key_path, value)
    
    def load_from_file(self, filename: str):
        """
        Carga configuración desde un archivo JSON.
        
        Args:
            filename: Nombre del archivo
        """
        try:
            with open(filename, 'r', encoding='utf-8') as f:
                file_config = json.load(f)
                self._deep_update(self.config, file_config)
        except (FileNotFoundError, json.JSONDecodeError) as e:
            print(f"Error cargando configuración: {e}")
    
    def save_to_file(self, filename: str):
        """
        Guarda la configuración actual a un archivo JSON.
        
        Args:
            filename: Nombre del archivo
        """
        try:
            with open(filename, 'w', encoding='utf-8') as f:
                json.dump(self.config, f, indent=2, ensure_ascii=False)
        except IOError as e:
            print(f"Error guardando configuración: {e}")
    
    def _deep_update(self, base_dict: Dict, update_dict: Dict):
        """
        Actualiza recursivamente un diccionario.
        
        Args:
            base_dict: Diccionario base
            update_dict: Diccionario con actualizaciones
        """
        for key, value in update_dict.items():
            if (key in base_dict and 
                isinstance(base_dict[key], dict) and 
                isinstance(value, dict)):
                self._deep_update(base_dict[key], value)
            else:
                base_dict[key] = value
    
    def get_robot_config(self, robot_type: str) -> Dict[str, Any]:
        """
        Obtiene configuración específica para un tipo de robot.
        
        Args:
            robot_type: Tipo de robot
            
        Returns:
            Configuración del robot
        """
        base_config = self.config.copy()
        robot_specific = self.get(f'robot_types.{robot_type}', {})
        
        # Combinar configuración base con específica del robot
        if robot_specific:
            base_config['robot'].update(robot_specific)
        
        return base_config
    
    def validate_config(self) -> List[str]:
        """
        Valida la configuración actual.
        
        Returns:
            Lista de errores encontrados
        """
        errors = []
        
        # Validar parámetros físicos
        wheel_radius = self.get('robot.wheel_radius')
        if wheel_radius is None or wheel_radius <= 0:
            errors.append("wheel_radius debe ser mayor que 0")
        
        wheel_base = self.get('robot.wheel_base')
        if wheel_base is None or wheel_base <= 0:
            errors.append("wheel_base debe ser mayor que 0")
        
        max_speed = self.get('robot.max_speed')
        if max_speed is None or max_speed <= 0:
            errors.append("max_speed debe ser mayor que 0")
        
        # Validar parámetros de sensores
        lidar_range = self.get('sensors.lidar.max_range')
        if lidar_range is None or lidar_range <= 0:
            errors.append("lidar max_range debe ser mayor que 0")
        
        # Validar parámetros de control
        pos_tolerance = self.get('control.navigation.position_tolerance')
        if pos_tolerance is None or pos_tolerance <= 0:
            errors.append("position_tolerance debe ser mayor que 0")
        
        return errors
    
    def reset_to_default(self):
        """Restablece la configuración a valores por defecto."""
        self.config = self.DEFAULT_CONFIG.copy()
    
    def print_config(self, section: Optional[str] = None):
        """
        Imprime la configuración actual.
        
        Args:
            section: Sección específica a imprimir (opcional)
        """
        if section:
            config_to_print = self.get(section, {})
            print(f"Configuración de {section}:")
        else:
            config_to_print = self.config
            print("Configuración completa:")
        
        print(json.dumps(config_to_print, indent=2, ensure_ascii=False))


# Instancia global de configuración
_global_config = None


def get_config() -> RobotConfig:
    """
    Obtiene la instancia global de configuración.
    
    Returns:
        Instancia de RobotConfig
    """
    global _global_config
    if _global_config is None:
        _global_config = RobotConfig()
    return _global_config


def set_config_file(filename: str):
    """
    Establece un archivo de configuración global.
    
    Args:
        filename: Archivo de configuración
    """
    global _global_config
    _global_config = RobotConfig(filename)
