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
        # Ejemplo de detección de obstáculos (específico del RosBot)
        if robot.obstacle_detected():
            print("Obstáculo detectado!")
            robot.turn_left()  # Método común
        else:
            robot.move_forward()  # Método común
        
        # Mostrar información cada cierto tiempo
        if iteration % 100 == 0:
            print(f"Iteración {iteration}:")
            robot.log_devices(to_terminal=True, to_file="./log_webots.log")
    
    # Limpiar recursos (método común)
    robot.cleanup()

if __name__ == "__main__":
    main()