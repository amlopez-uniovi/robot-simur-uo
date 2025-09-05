"""
Controlador base para la demo de navegación aleatoria (EPuck y RosBot)
Estructura unificada: inicialización, bucle principal, sensores, control y logging
"""
from robot_simur_uo.controllers.random_navigation import RandomNavigationController
import math

def run_random_navigation_demo(RobotClass):
    """
    Ejecuta una demo de navegación aleatoria para un robot dado.

    Args:
        RobotClass (type): Clase del robot a instanciar (debe implementar la interfaz de Webots).

    Ejemplo:
        >>> from random_navigation_demo_controller import run_random_navigation_demo
        >>> from robot_simur_uo.webots.epuck_robot import EPuck
        >>> run_random_navigation_demo(EPuck)
    """
    robot = RobotClass()
    # Crear controlador de navegación aleatoria
    controller = RandomNavigationController(
        workspace_bounds=(-1.9, 1.9, -1.9, 1.9),
        linear_gain=0.8,
        steering_gain=2.5,
        goal_tolerance=0.1
    )
    iteration_count = 0
    # Bucle principal de control
    while robot.step() != -1:
        iteration_count += 1
        # Obtener posición y orientación actuales usando el controlador de navegación
        current_x, current_y, current_angle = controller.get_robot_state(robot)
        # Actualizar controlador y obtener comandos
        drive_speed, steering_speed = controller.update(
            robot, current_x, current_y, current_angle
        )
        # Aplicar comandos al robot
        robot.set_drive_command(drive_speed, steering_speed)
        # Mostrar progreso básico cada 100 iteraciones
        if iteration_count % 100 == 0:
            stats = controller.get_statistics()
            if controller.current_goal:
                goal_distance = math.sqrt(
                    (controller.current_goal[0] - current_x)**2 + 
                    (controller.current_goal[1] - current_y)**2
                )
                print(f"Iteración {iteration_count}: Pos=({current_x:.2f}, {current_y:.2f}), "
                        f"Objetivo=({controller.current_goal[0]:.2f}, {controller.current_goal[1]:.2f}), "
                        f"Distancia={goal_distance:.2f}m, Objetivos={stats['goals_reached']}")
            print("=" * 60)