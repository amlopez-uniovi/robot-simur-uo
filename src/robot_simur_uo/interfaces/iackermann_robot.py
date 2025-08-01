"""
Interfaz específica para robots con dirección tipo Ackermann.
"""

from .irobot_base import IRobotBase


class IAckermannRobot(IRobotBase):
    """
    Interfaz específica para robots con dirección tipo Ackermann.
    
    Los robots Ackermann usan directamente los métodos de la interfaz base:
    - set_drive_speed() / get_drive_speed()
    - set_steering_angle() / get_steering_angle()
    - step(), get_pose(), set_pose(), stop(), cleanup()
    
    Esta clase existe para:
    1. Claridad semántica en el tipo de robot
    2. Consistencia con IDifferentialRobot  
    3. Posibles extensiones futuras específicas de Ackermann
    
    No redefine métodos ya que IRobotBase implementa directamente el modelo Ackermann.
    """
    pass
