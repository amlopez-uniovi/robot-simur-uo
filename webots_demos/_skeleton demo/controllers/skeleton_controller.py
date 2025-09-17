from robot_simur_uo.controllers.navigation import NavigationController
import time

def run_skeleton_demo(RobotClass):
    """
    Ejecuta una demo básica de navegación para un robot dado.

    Args:
        RobotClass (type): Clase del robot a instanciar (debe implementar la interfaz de Webots).

    Ejemplo:
        >>> from basic_navigation_controller import run_basic_demo
        >>> from robot_simur_uo.webots.epuck_robot import EPuck
        >>> run_basic_demo(EPuck)
    """
    robot = RobotClass()
    while robot.step() != -1:
   
        # Avanzar un segundo
        robot.set_drive_command(10.0, 100.0)    

    robot.cleanup()