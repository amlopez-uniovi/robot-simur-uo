# Este archivo contiene el controlador principal del robot e-puck
# Utiliza la clase EPuck refactorizada que hereda de BaseRobot

import sys
import os

from robot_simur_uo import BaseRobot, EPuck, NavigationController

def main():
    # Crear instancia del robot e-puck
    robot = EPuck()
    
    # Ejemplo de uso del controlador de navegación
    controller = NavigationController(max_speed=1.0, linear_gain = 5.0, angular_gain = 2.0)
    controller.set_target(1.0, 0.5, tol=0.1)
        
    # Ejemplo de uso de métodos comunes y específicos
    while robot.step() != -1:
        # Obtener posición GPS (método común)
        gps_position = robot.get_gps_position()
        
        # Obtener orientación de la brújula (método común)
        compass_direction, compass_angle = robot.get_compass_orientation()
        
        current_x = gps_position[0]
        current_y = gps_position[1]
        current_angle = compass_angle
        
        if controller.is_target_reached(current_x, current_y):
            print("¡Objetivo alcanzado!")
            robot.set_motor_velocities(0, 0)  # Detener el robot
            break
        
        left_speed, right_speed = controller.calculate_motor_speeds(
            current_x, current_y, current_angle
        )
        print(f"Posición: ({current_x:.2f}, {current_y:.2f}), "
                f"Ángulo: {current_angle:.2f}, "
                f"Velocidades: Izq={left_speed:.2f}, Der={right_speed:.2f}")

        robot.set_motor_velocities(left_speed, right_speed)
            
        # Mostrar información cada cierto tiempo
        if robot.step(0) % 1000 == 0:
            print(f"GPS: {gps_position}")
            print(f"Brújula: {compass_angle:.2f}°")
    
    # Limpiar recursos (método común)
    robot.cleanup()

if __name__ == "__main__":
    main()
