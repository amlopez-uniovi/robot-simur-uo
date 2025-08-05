# Controlador Bug0 simple para el robot e-puck

import math
from robot_simur_uo import EPuck
from robot_simur_uo.controllers.navigation_bug0 import Bug0NavigationController


def main():
    """Controlador Bug0 simple para EPuck."""
    print("EPuck - Controlador Bug0 Simple")
    
    # Robot y controlador Bug0
    robot = EPuck()
    controller = Bug0NavigationController()
    
    # Objetivo
    goal_x, goal_y = 1.0, 1.0
    controller.set_target(goal_x, goal_y)
    
    print(f"Objetivo: ({goal_x}, {goal_y})")
    print(f"Umbral obstáculo: {controller.obstacle_threshold}m")
    
    iteration = 0
    
    # Bucle principal
    while robot.step() != -1:
        iteration += 1
        
        # Posición y orientación actuales
        pos = robot.get_gps_position()
        x, y = pos[0], pos[1]
        _, angle = robot.get_compass_orientation()
        # Obtener sensores de obstáculos usando solo 5 sectores
        obstacle_sensors = robot.get_obstacle_sensors(num_sectors=5)
        
        # DEBUG: Mostrar todos los sectores en las primeras iteraciones
        #El Lidar de EPuck barre clockwise de -pi a pi
        if iteration <= 1000:
            print(f"DEBUG Iter {iteration} - Sectores (5):")
            sector_names = ["Izquierda", "Frontal Izquierdo", "Frontal", "Frontal Derecho", "Derecha"]
            for i, dist in enumerate(obstacle_sensors):
                print(f"   Sector {i} ({sector_names[i]}): {dist:.3f}m")
            print(f"   LiDAR raw data points: {len(robot.get_lidar_data()) if robot.get_lidar_data() else 0}")
        
        # Calcular comandos con Bug0
        drive, steer = controller.calculate_control_commands(x, y, angle, min(obstacle_sensors[1:4]))

        # Aplicar comandos al robot
        robot.set_drive_command(drive, steer)        
            
        # Verificar si llegamos al objetivo
        if controller.is_target_reached(x, y):
            print("Objetivo alcanzado")
            robot.stop()
            break
    
    robot.cleanup()
    print("Controlador Bug0 simple finalizado")


if __name__ == "__main__":
    main()
