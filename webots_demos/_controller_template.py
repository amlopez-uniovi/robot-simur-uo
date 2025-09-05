"""
Controlador base para la demo de test de sensores (EPuck y RosBot)
Estructura unificada: inicialización, bucle principal, lectura de sensores y logging
"""

import math

def run_controller_name(RobotClass):
    robot = RobotClass()

    iteration = 0
    print("Iniciando demo...")

    while robot.step() != -1:
        iteration += 1
        # --- Lectura de sensores ---
        gps_pos = robot.get_gps_position()
        compass_dir, compass_angle = robot.get_compass_orientation()
        # Otros sensores según el robot...

        # --- Lógica de control ---
        # Ejemplo: avanzar si no hay obstáculo
        robot.move_forward()

        robot.log_devices(to_terminal=True)
        
        # --- Logging periódico ---
        if iteration % 10 == 0:
            break
            

    robot.cleanup()
    robot.stop()
