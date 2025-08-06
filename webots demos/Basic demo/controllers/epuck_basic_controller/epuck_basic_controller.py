# Este archivo contiene el controlador principal del robot e-puck
# Utiliza la clase EPuck refactorizada que hereda de WebotsBaseDifferentialRobot

import sys
import os

from robot_simur_uo import WebotsBaseDifferentialRobot, EPuck

def main():
    # Crear instancia del robot e-puck
    robot = EPuck()
        
    iteration = 0    
    # Ejemplo de uso de métodos comunes y específicos
    while robot.step() != -1:
        iteration = iteration + 1
        # Detección de obstáculos usando sensores frontales del e-puck (ps0 y ps7 suelen ser los frontales extremos)
        distance_values = robot.get_distance_sensor_values()
        front_left = distance_values[0]  # ps0
        front_right = distance_values[7] # ps7
        threshold = 80.0  # Ajusta este valor según la escala de los sensores del e-puck
        if front_left > threshold or front_right > threshold:
            print("Obstáculo detectado!")
            robot.turn_left()
        else:
            robot.move_forward()
        # Mostrar información cada cierto tiempo
        if iteration % 100 == 0:
            print(f"Iteración {iteration}:")
            robot.log_devices(to_terminal=True, to_file="./log_webots.log")
    
    # Limpiar recursos (método común)
    robot.cleanup()

if __name__ == "__main__":
    main()
