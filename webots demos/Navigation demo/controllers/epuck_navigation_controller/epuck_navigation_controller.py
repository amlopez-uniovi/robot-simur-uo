# Este archivo contiene el controlador principal del robot e-puck
# Utiliza la clase EPuck refactorizada que hereda de WebotsBaseDifferentialRobot

import sys
import os

from robot_simur_uo import WebotsBaseDifferentialRobot, EPuck, NavigationController

def main():
    # Crear instancia del robot e-puck
    robot = EPuck()
    
    # Ejemplo de uso del controlador de navegación con parámetros muy conservadores
    controller = NavigationController(linear_gain=0.3, steering_gain=0.5)
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
            robot.set_drive_command(0.0, 0.0)  # Detener usando interfaz unificada
            break
        
        # Usar comandos Ackermann unificados
        drive_speed, steering_speed = controller.calculate_control_commands(
            current_x, current_y, current_angle
        )
        
        # Debug: Mostrar velocidades antes de aplicarlas
        print(f"Posición: ({current_x:.2f}, {current_y:.2f}) → Destino: ({controller.target_x:.2f}, {controller.target_y:.2f})")
        print(f"Ángulo actual: {current_angle:.2f}, Comandos: Velocidad={drive_speed:.2f}, Velocidad giro={steering_speed:.2f}")
        
        # Obtener velocidades de motores después de la conversión para debug
        robot.set_drive_command(drive_speed, steering_speed)
        left_speed, right_speed = robot.get_motor_speeds()
        print(f"Velocidades motores: Izq={left_speed:.2f}, Der={right_speed:.2f}")
        print("---")
            
        # Mostrar información cada cierto tiempo
        if robot.step(0) % 1000 == 0:
            print(f"GPS: {gps_position}")
            print(f"Brújula: {compass_angle:.2f}°")
    
    # Limpiar recursos (método común)
    robot.cleanup()

if __name__ == "__main__":
    main()
