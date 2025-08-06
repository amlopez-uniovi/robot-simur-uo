# Este archivo contiene el controlador principal del robot RosBot
# Utiliza la clase RosBot refactorizada que hereda de WebotsBaseDifferentialRobot

import sys
import os

from robot_simur_uo import WebotsBaseDifferentialRobot, RosBot

def main():
    # Crear instancia del robot RosBot
    robot = RosBot()
        
    iteration = 0    
    # Ejemplo de uso de métodos comunes y específicos
    while robot.step() != -1:
        iteration = iteration + 1
        # Detección de obstáculos usando sensores frontales (fl_range y fr_range)
        distance_values = robot.get_distance_sensor_values()
        front_left = distance_values[0]  # fl_range
        front_right = distance_values[2] # fr_range
        threshold = 0.8
        if front_left < threshold or front_right < threshold:
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