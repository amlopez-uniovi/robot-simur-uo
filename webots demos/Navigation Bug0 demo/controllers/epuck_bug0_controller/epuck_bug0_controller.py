# Controlador Bug0 simple para el robot e-puck

import math
from robot_simur_uo import EPuck
from robot_simur_uo.controllers.navigation_bug0 import Bug0NavigationController


def main():
    """Controlador Bug0 simple para EPuck."""
    print("🐛 EPuck - Controlador Bug0 Simple")
    
    # Robot y controlador Bug0
    robot = EPuck()
    controller = Bug0NavigationController()
    
    # Objetivo
    goal_x, goal_y = 1.0, 1.0
    controller.set_target(goal_x, goal_y)
    
    print(f"🎯 Objetivo: ({goal_x}, {goal_y})")
    print(f"🔧 Umbral obstáculo: {controller.obstacle_threshold}m")
    
    iteration = 0
    
    # Bucle principal
    while robot.step() != -1:
        iteration += 1
        
        # Posición y orientación actuales
        pos = robot.get_gps_position()
        x, y = pos[0], pos[1]
        _, angle = robot.get_compass_orientation()
        
        # Obtener sensores de obstáculos usando solo 3 sectores
        obstacle_sensors = robot.get_obstacle_sensors(num_sectors=3)
        
        # DEBUG: Mostrar todos los sectores en las primeras iteraciones
        if iteration <= 5:
            print(f"🔍 DEBUG Iter {iteration} - Sectores (3):")
            sector_names = ["Izquierda", "Frontal", "Derecha"]
            for i, dist in enumerate(obstacle_sensors):
                print(f"   Sector {i} ({sector_names[i]}): {dist:.3f}m")
            print(f"   LiDAR raw data points: {len(robot.get_lidar_data()) if robot.get_lidar_data() else 0}")
        
        # Calcular comandos con Bug0
        drive, steer = controller.calculate_control_commands(x, y, angle, obstacle_sensors)
        
        # PROTECCIÓN ANTI-CHOQUES: Solo considerar sector frontal (sector 1)
        front_obstacle = obstacle_sensors[1] if len(obstacle_sensors) > 1 else 1.0  # Sector frontal
        
        # Filtrar valores sospechosos (muy pequeños que pueden ser ruido)
        def filter_sensor_value(value):
            if value < 0.05:  # Valores muy pequeños probablemente son ruido
                return 1.0
            return value
        
        front_obstacle = filter_sensor_value(front_obstacle)
        
        if front_obstacle < 0.5:  # Obstáculo a menos de 50cm en sector frontal
            drive = 0.0  # Parar completamente
            steer = 1.5   # Girar fuerte a la izquierda
            if iteration % 10 == 0:  # Debug cada 10 iteraciones para no saturar
                print(f"   🚨 PARADA DE EMERGENCIA: obstáculo frontal a {front_obstacle:.2f}m")
        
        # Aplicar comandos al robot
        robot.set_drive_command(drive, steer)
        
        # Debug cada 50 iteraciones
        if iteration <= 10 or iteration % 50 == 0:
            distance = math.sqrt((goal_x - x)**2 + (goal_y - y)**2)
            
            print(f"📍 Iter {iteration}: pos=({x:.2f},{y:.2f}) dist={distance:.2f}m")
            print(f"   Obstáculo frontal: {front_obstacle:.2f}m")
            print(f"   Comandos: vel={drive:.2f}, giro={steer:.2f}")
            
            if front_obstacle < controller.obstacle_threshold:
                print(f"   🚧 OBSTÁCULO FRONTAL DETECTADO - Bug0 activado")
        
        # Verificar si llegamos al objetivo
        if controller.is_target_reached(x, y):
            print("🎯 ¡Objetivo alcanzado!")
            robot.stop()
            break
    
    robot.cleanup()
    print("✅ Controlador Bug0 simple finalizado")


if __name__ == "__main__":
    main()
