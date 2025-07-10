# Este archivo contiene el controlador principal del robot e-puck
# Utiliza la clase EPuck refactorizada que hereda de BaseRobot

import sys
import os

from robot_simur_uo import BaseRobot, EPuck

def main():
    # Crear instancia del robot e-puck
    robot = EPuck()
        
    # Ejemplo de uso de métodos comunes y específicos
    while robot.step() != -1:
        # Obtener posición GPS (método común)
        gps_position = robot.get_gps_position()
        
        # Obtener orientación de la brújula (método común)
        compass_direction, compass_angle = robot.get_compass_orientation()
        
        # Obtener sensores de distancia (específico del e-puck)
        distance_sensors = robot.get_distance_sensor_values()
        
        # Obtener datos del lidar (método común heredado de BaseRobot)
        lidar_data = robot.get_lidar_data()
        lidar_closest_obstacle = robot.get_lidar_closest_obstacle()
        
        # Mostrar información avanzada del lidar (ahora también común)
        if robot.step(0) % 5000 == 0:  # Cada 5 segundos
            robot.print_lidar_summary()  # Método común
            robot.print_lidar_point_cloud(5)  # Método común
        
        # Ejemplo de detección de obstáculos (específico del e-puck)
        if robot.obstacle_detected():
            print("Obstáculo detectado!")
            robot.turn_left()  # Método común
        else:
            robot.move_forward()  # Método común
        
        # Mostrar información cada cierto tiempo
        if robot.step(0) % 1000 == 0:
            print(f"GPS: {gps_position}")
            print(f"Brújula: {compass_angle:.2f}°")
            print(f"Sensores distancia: {distance_sensors}")
            print(f"Lidar puntos: {len(lidar_data) if lidar_data else 0}")
            print(f"Obstáculo más cercano: {lidar_closest_obstacle:.3f}m")
    
    # Limpiar recursos (método común)
    robot.cleanup()

if __name__ == "__main__":
    main()
