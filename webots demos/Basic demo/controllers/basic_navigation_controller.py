from robot_simur_uo.controllers.navigation import NavigationController

def run_basic_demo(RobotClass):
    robot = RobotClass()
    controller = NavigationController(linear_gain=1.0, steering_gain=2.0)
    controller.set_target(-1.0, 0.5, tol=0.1)
    iteration = 0
    print(f"Iniciando demo básica para {RobotClass.__name__}...")

    while robot.step() != -1:
        iteration += 1
        gps_position = robot.get_gps_position()
        _, compass_angle = robot.get_compass_orientation()
        current_x = gps_position[0]
        current_y = gps_position[1]
        current_angle = compass_angle

        if controller.is_target_reached(current_x, current_y):
            print("¡Objetivo alcanzado!")
            robot.set_drive_command(0.0, 0.0)
            break

        drive_speed, steering_speed = controller.calculate_control_commands(
            current_x, current_y, current_angle
        )
        robot.set_drive_command(drive_speed, steering_speed)

        print(f"Posición: ({current_x:.2f}, {current_y:.2f}) → Destino: ({controller.target_x:.2f}, {controller.target_y:.2f})")
        print(f"Ángulo actual: {current_angle:.2f}, Comandos: Velocidad={drive_speed:.2f}, Velocidad giro={steering_speed:.2f}")

        if iteration % 100 == 0:
            robot.log_devices(to_terminal=True)

    robot.cleanup()