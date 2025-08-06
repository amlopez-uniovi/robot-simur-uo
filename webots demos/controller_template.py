"""
Plantilla base para controladores de robots Webots
- Inicialización de robot
- Bucle principal
- Acceso a sensores y actuadores
- Logging y estructura uniforme
"""

def main():
    # Importar la clase de robot adecuada
    from robot_simur_uo.webots.rosbot_robot import RosBot  # O EPuck, según el demo
    robot = RosBot()  # O EPuck()

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

        # --- Logging periódico ---
        if iteration % 100 == 0:
            print(f"Iteración {iteration}:")
            robot.log_devices(to_terminal=True)

if __name__ == "__main__":
    main()
