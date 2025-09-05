"""
Controlador base para la demo de test de sensores (EPuck y RosBot)
Estructura unificada: inicialización, bucle principal, lectura de sensores y logging
"""

import math

from robot_simur_uo.controllers.navigation_potential_field import NavigationPotentialFieldController
import numpy as np
from robot_simur_uo.utils.coordinates import polar_to_cartesian, transform_points


def run_navigation_potential_field(RobotClass):
    """
    Ejecuta una demo de navegación con campos potenciales para un robot dado.

    Args:
        RobotClass (type): Clase del robot a instanciar (debe implementar la interfaz de Webots).

    Ejemplo:
        >>> from navigation_potential_field_controller import run_navigation_potential_field
        >>> from robot_simur_uo.webots.epuck_robot import EPuck
        >>> run_navigation_potential_field(EPuck)
    """
    robot = RobotClass()
    controller = NavigationPotentialFieldController(linear_gain=1.0, steering_gain=2.0 , 
                                                   attraction_gain=1.0, repulsion_gain=4.0, repulsion_threshold=0.8)
    # Destino fijo (puedes parametrizarlo)
    target_x, target_y = 1.9, 0.0
    controller.set_target(target_x, target_y)
    iteration = 0
    print("Iniciando demo...")

    while robot.step() != -1:
        iteration += 1
        # Obtener pose del robot usando método estándar
        pose = robot.get_pose()  # Devuelve RobotPose (x, y, theta)
        robot_x, robot_y, robot_theta = pose.x, pose.y, pose.theta
        print(f"[Iteración {iteration}] Pose: x={robot_x:.3f}, y={robot_y:.3f}, theta={robot_theta:.3f}")

        # Obtener obstáculo más cercano usando LidarManager
        lidar_manager = robot.get_lidar_manager()
        if lidar_manager is not None:
            angle_min = -math.pi            
            angle_max = math.pi
            _, _, min_angle, min_distance = lidar_manager.get_closest_obstacle_in_angle_range(angle_min, angle_max)
            print(f"[Iteración {iteration}] Obstáculo local: ángulo={min_angle:.3f}, distancia={min_distance:.3f}")
            local_obs_point = polar_to_cartesian([(min_angle, min_distance)])
            global_obs_point = transform_points(local_obs_point, (robot_x, robot_y, robot_theta))
            if global_obs_point:
                obstacle_x, obstacle_y = global_obs_point[0]
                print(f"[Iteración {iteration}] Obstáculo global: x={obstacle_x:.3f}, y={obstacle_y:.3f}")
            else:
                obstacle_x, obstacle_y = float('inf'), float('inf')
                print(f"[Iteración {iteration}] No hay obstáculo válido, usando pose del robot.")
        else:
            obstacle_x, obstacle_y = float('inf'), float('inf')
            print(f"[Iteración {iteration}] LidarManager no disponible, usando pose del robot.")

        drive_speed, steering_speed = controller.calculate_control_commands(
            robot_x, robot_y, robot_theta, obstacle_x, obstacle_y)

        print(f"[Iteración {iteration}] Comandos: drive_speed={drive_speed:.3f}, steering_speed={steering_speed:.3f}")

        robot.set_drive_command(drive_speed, steering_speed)

        if controller.is_target_reached(robot_x, robot_y):
            print("¡Objetivo alcanzado!")
            robot.stop()
            break   
        
        #robot.log_devices(to_terminal=True)

        
    print("Demo finalizada. Limpiando recursos...")
    robot.cleanup()
    robot.stop()
