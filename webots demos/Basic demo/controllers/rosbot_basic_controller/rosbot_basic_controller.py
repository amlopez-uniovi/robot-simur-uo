# Este archivo contiene el controlador principal del robot RosBot
# Utiliza la clase RosBot refactorizada que hereda de BaseRobot

import sys
import os

from robot_simur_uo import BaseRobot, RosBot
def main():
    # Crear instancia del robot RosBot
    robot = RosBot()
    
    print("Robot RosBot inicializado correctamente")
    print("Funcionalidades disponibles:")
    print("- Métodos comunes heredados de BaseRobot: step(), get_gps_position(), get_compass_orientation(), move_forward(), etc.")
    print("- Métodos específicos del RosBot: get_accelerometer_values(), get_camera_rgb_image(), get_imu_compass_values(), etc.")
    print("- Métodos del lidar ahora comunes: get_lidar_fov(), print_lidar_summary(), etc.")
    
    # Ejemplo de uso de métodos comunes y específicos
    while robot.step() != -1:
        # Obtener posición GPS (método común)
        gps_position = robot.get_gps_position()
        
        # Obtener orientación de la brújula (método común)
        compass_direction, compass_angle = robot.get_compass_orientation()
        
        # Obtener sensores de distancia (específico del RosBot - 4 sensores)
        distance_sensors = robot.get_distance_sensor_values()
        
        # Obtener datos del acelerómetro (específico del RosBot)
        accelerometer_values = robot.get_accelerometer_values()
        
        # Obtener datos del giroscopio (específico del RosBot)
        gyro_values = robot.get_gyro_values()
        
        # Obtener datos del LiDAR (método común heredado de BaseRobot)
        lidar_data = robot.get_lidar_data()
        lidar_closest_obstacle = robot.get_lidar_closest_obstacle()
        
        # Mostrar información avanzada del lidar (ahora también común)
        if robot.step(0) % 5000 == 0:  # Cada 5 segundos
            robot.print_lidar_summary()  # Método común
            robot.print_lidar_point_cloud(5)  # Método común
        
        # Obtener posiciones de las ruedas (específico del RosBot)
        wheel_positions = robot.get_position_sensor_values()
        
        # Ejemplo de navegación básica
        front_left_distance = distance_sensors[0] if distance_sensors else float('inf')
        front_right_distance = distance_sensors[2] if distance_sensors else float('inf')
        
        # Detección de obstáculos simple
        if front_left_distance < 0.5 or front_right_distance < 0.5:
            print("Obstáculo detectado!")
            robot.turn_right()  # Método común
        else:
            robot.move_forward()  # Método común
        
        # Mostrar información cada cierto tiempo
        if robot.step(0) % 1000 == 0:
            print(f"GPS: {gps_position}")
            print(f"Brújula: {compass_angle:.2f}°")
            print(f"Sensores distancia: {distance_sensors}")
            print(f"Acelerómetro: {accelerometer_values}")
            print(f"Giroscopio: {gyro_values}")
            print(f"Lidar puntos: {len(lidar_data) if lidar_data else 0}")
            print(f"Obstáculo más cercano: {lidar_closest_obstacle:.3f}m")
            print(f"Posiciones ruedas: {wheel_positions}")
    
    # Limpiar recursos (método común)
    robot.cleanup()

if __name__ == "__main__":
    main()