# Controlador Bug0 simple para el robot RosBot

import math
from robot_simur_uo import RosBot
from robot_simur_uo.controllers.navigation_bug0 import Bug0NavigationController


def main():
    """Controlador Bug0 simple para RosBot."""
    print("🐛 RosBot - Controlador Bug0 Simple")
    
    # Robot y controlador Bug0
    robot = RosBot()
    controller = Bug0NavigationController()
    
    # Objetivo
    goal_x, goal_y = 2.0, 2.0
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
        
        # Obtener sensores de obstáculos con parámetros más restrictivos para RosBot
        # El RosBot tiene LiDAR más sensible, necesitamos rangos más estrictos
        obstacle_sensors = robot.get_lidar_sectored_distances(
            num_sectors=3,
            min_range=0.15,    # Rango mínimo más alto (15cm vs 5cm)
            max_range=3.0      # Rango máximo más bajo (3m vs 10m)
        )
        
        # DEBUG: Mostrar todos los sectores en las primeras iteraciones
        if iteration <= 5:
            print(f"🔍 DEBUG Iter {iteration} - Sectores (3) con rangos restrictivos:")
            sector_names = ["Izquierda", "Frontal", "Derecha"]
            for i, dist in enumerate(obstacle_sensors):
                print(f"   Sector {i} ({sector_names[i]}): {dist:.3f}m")
            raw_data = robot.get_lidar_data()
            print(f"   LiDAR raw data points: {len(raw_data) if raw_data else 0}")
            if raw_data:
                # Mostrar solo algunos valores del LiDAR raw para diagnóstico
                sample_indices = [0, len(raw_data)//4, len(raw_data)//2, 3*len(raw_data)//4, len(raw_data)-1]
                sample_values = [raw_data[i] for i in sample_indices if i < len(raw_data)]
                print(f"   LiDAR samples: {[f'{v:.2f}' for v in sample_values]}")
        
        # Calcular comandos con Bug0
        drive, steer = controller.calculate_control_commands(x, y, angle, obstacle_sensors)
        
        # PROTECCIÓN ANTI-CHOQUES: Solo considerar sector frontal (sector 1)
        front_obstacle = obstacle_sensors[1] if len(obstacle_sensors) > 1 else 1.0  # Sector frontal
        
        # Filtrar valores sospechosos - más agresivo para RosBot
        def filter_sensor_value(value):
            # Para RosBot, filtrar más agresivamente
            if value < 0.2:  # Ignorar valores menores a 20cm (vs 5cm en EPuck)
                return 3.0   # Considerar como "sin obstáculo" (vs 1.0m)
            return value
        
        front_obstacle = filter_sensor_value(front_obstacle)
        
        # Umbral más conservador para RosBot
        emergency_threshold = 0.3  # 30cm vs 50cm del EPuck
        
        if front_obstacle < emergency_threshold:  # Obstáculo muy cerca
            drive = 0.0  # Parar completamente
            steer = 1.5   # Girar fuerte a la izquierda
            if iteration % 10 == 0:  # Debug cada 10 iteraciones para no saturar
                print(f"   🚨 PARADA DE EMERGENCIA: obstáculo frontal a {front_obstacle:.2f}m (umbral: {emergency_threshold}m)")
        
        # Aplicar comandos al robot
        robot.set_drive_command(drive, steer)
        
        # Debug cada 50 iteraciones
        if iteration <= 10 or iteration % 50 == 0:
            distance = math.sqrt((goal_x - x)**2 + (goal_y - y)**2)
            
            print(f"📍 Iter {iteration}: pos=({x:.2f},{y:.2f}) dist={distance:.2f}m")
            print(f"   Obstáculo frontal: {front_obstacle:.2f}m (filtrado)")
            print(f"   Comandos: vel={drive:.2f}, giro={steer:.2f}")
            print(f"   Umbral Bug0: {controller.obstacle_threshold}m, Emergencia: {emergency_threshold}m")
            
            if front_obstacle < emergency_threshold:
                print(f"   🚧 OBSTÁCULO FRONTAL DETECTADO - Parada de emergencia")
            elif front_obstacle < controller.obstacle_threshold:
                print(f"   � OBSTÁCULO CERCANO - Bug0 activado")
        
        # Verificar si llegamos al objetivo
        if controller.is_target_reached(x, y):
            print("🎯 ¡Objetivo alcanzado!")
            robot.stop()
            break
    
    robot.cleanup()
    print("✅ Controlador Bug0 simple finalizado")


if __name__ == "__main__":
    main()



if __name__ == "__main__":
    main()
