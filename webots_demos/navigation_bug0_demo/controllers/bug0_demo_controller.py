"""
Controlador base para la demo de navegación Bug0 (EPuck y RosBot)
Estructura unificada: inicialización, bucle principal, sensores, control y logging
"""
from robot_simur_uo.controllers.navigation_bug0 import Bug0NavigationController
import numpy as np

def run_bug0_demo(RobotClass):
    """
    Ejecuta una demo de navegación Bug0 para un robot dado.

    Args:
        RobotClass (type): Clase del robot a instanciar (debe implementar la interfaz de Webots).

    Ejemplo:
        >>> from bug0_demo_controller import run_bug0_demo
        >>> from robot_simur_uo.webots.epuck_robot import EPuck
        >>> run_bug0_demo(EPuck)
    """
    robot = RobotClass()
    controller = Bug0NavigationController(linear_gain=1.0, steering_gain=4.0)
    controller.set_target(1.0, 1, tol=0.1)
    iteration = 0
    print(f"Iniciando demo Bug0 para {RobotClass.__name__}...")

    while robot.step() != -1:
        iteration += 1

        pose = robot.get_pose()
        current_x, current_y, current_angle = pose.x, pose.y, pose.theta  
                  
        if controller.is_target_reached(current_x, current_y):
            print("¡Objetivo alcanzado!")
            robot.set_drive_command(0.0, 0.0)
            break

        front_distance = None
        if hasattr(robot, 'lidar_manager'):
            _, _, _, front_distance = robot.lidar_manager.get_closest_obstacle_in_angle_range(-np.pi/4, np.pi/4)

        drive_speed, steering_speed = controller.calculate_control_commands(
            current_x, current_y, current_angle, front_distance
        )
        robot.set_drive_command(drive_speed, steering_speed)

        print(f"Posición: ({current_x:.2f}, {current_y:.2f}) → Destino: ({controller.target_x:.2f}, {controller.target_y:.2f})")
        print(f"Ángulo actual: {current_angle:.2f}, Comandos: Velocidad={drive_speed:.2f}, Velocidad giro={steering_speed:.2f}")

        if iteration % 100 == 0:
            robot.log_devices(to_terminal=True)

    robot.cleanup()
