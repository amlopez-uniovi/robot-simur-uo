"""
Factory para crear instancias de robots de manera sencilla.
"""

from typing import Dict, Type, Any
from .base_robot import BaseRobot
from .epuck_robot import EPuck
from .rosbot_robot import RosBot


class RobotFactory:
    """Factory para crear robots de diferentes tipos."""
    
    _robot_types: Dict[str, Type[BaseRobot]] = {
        'epuck': EPuck,
        'e-puck': EPuck,
        'rosbot': RosBot,
        'ros-bot': RosBot,
    }
    
    @classmethod
    def create_robot(cls, robot_type: str, **kwargs) -> BaseRobot:
        """
        Crea una instancia de robot del tipo especificado.
        
        Args:
            robot_type: Tipo de robot ('epuck', 'rosbot', etc.)
            **kwargs: Argumentos adicionales para el constructor del robot
            
        Returns:
            Instancia del robot creado
            
        Raises:
            ValueError: Si el tipo de robot no está soportado
        """
        robot_type_lower = robot_type.lower()
        
        if robot_type_lower not in cls._robot_types:
            available_types = list(cls._robot_types.keys())
            raise ValueError(
                f"Tipo de robot '{robot_type}' no soportado. "
                f"Tipos disponibles: {available_types}"
            )
        
        robot_class = cls._robot_types[robot_type_lower]
        return robot_class(**kwargs)
    
    @classmethod
    def register_robot_type(cls, name: str, robot_class: Type[BaseRobot]):
        """
        Registra un nuevo tipo de robot en la factory.
        
        Args:
            name: Nombre del tipo de robot
            robot_class: Clase del robot a registrar
        """
        cls._robot_types[name.lower()] = robot_class
    
    @classmethod
    def get_available_types(cls) -> list:
        """Retorna la lista de tipos de robots disponibles."""
        return list(cls._robot_types.keys())


def create_robot(robot_type: str, **kwargs) -> BaseRobot:
    """
    Función conveniente para crear robots.
    
    Args:
        robot_type: Tipo de robot a crear
        **kwargs: Argumentos adicionales
        
    Returns:
        Instancia del robot
    """
    return RobotFactory.create_robot(robot_type, **kwargs)
