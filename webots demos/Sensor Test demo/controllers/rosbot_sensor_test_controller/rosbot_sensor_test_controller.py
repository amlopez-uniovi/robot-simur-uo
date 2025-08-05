# Controlador de test de sensores para RosBot - Giro 360° sobre sí mismo
# Gira 360° usando turn_left() mientras testea todos los sensores

import math
from robot_simur_uo import RosBot


def main():
    """Controlador de test de sensores girando 360° sobre sí mismo para RosBot."""
    print("🔬 RosBot - Test de Sensores con Giro 360°")
    
    # Robot RosBot
    robot = RosBot()
    
    print("🎯 Test: Robot girando 360° sobre sí mismo")
    print("🔄 Usando método turn_left() para rotación continua")
    
    iteration = 0
    start_angle = None
    total_rotation = 0.0
    
    # Bucle principal
    while robot.step() != -1:
        iteration += 1
        
        # Obtener posición y orientación actuales
        pos = robot.get_gps_position()
        current_x, current_y = pos[0], pos[1]
        _, current_angle = robot.get_compass_orientation()
        
        # Calcular comandos de navegación
        drive_speed, steering_speed = controller.calculate_control_commands(
            current_x, current_y, current_angle
        )
        
        # Aplicar comandos al robot
        robot.set_drive_command(drive_speed, steering_speed)
        
        # TEST DE SENSORES - Mostrar información cada 50 iteraciones
        if iteration % 50 == 0:
            print(f"\n📍 === ITERACIÓN {iteration} ===")
            print(f"   Posición actual: ({current_x:.3f}, {current_y:.3f})")
            print(f"   Waypoint actual: {controller.current_waypoint_index}")
            if controller.current_waypoint_index < len(waypoints.waypoints):
                wp = waypoints.waypoints[controller.current_waypoint_index]
                print(f"   Waypoint objetivo: ({wp[0]:.3f}, {wp[1]:.3f})")
            print(f"   Ángulo actual: {math.degrees(current_angle):.1f}°")
            print(f"   Comandos: vel={drive_speed:.3f}, giro={steering_speed:.3f}")
            
            # TEST: Sensores de obstáculos con parámetros restrictivos para RosBot
            obstacle_sensors = robot.get_lidar_sectored_distances(
                num_sectors=8,
                min_range=0.15,  # Filtrado más agresivo para RosBot
                max_range=3.0
            )
            print(f"   🔍 Sensores obstáculos (8 sectores, filtrados):")
            for i, dist in enumerate(obstacle_sensors[:4]):  # Mostrar solo 4 primeros
                print(f"      Sector {i}: {dist:.3f}m")
            
            # TEST: Datos LiDAR raw
            lidar_data = robot.get_lidar_data()
            if lidar_data:
                # Filtrar valores válidos
                valid_ranges = [d for d in lidar_data if d > 0.1 and d < 10.0 and d != float('inf')]
                if valid_ranges:
                    min_distance = min(valid_ranges)
                    max_distance = max(valid_ranges)
                    avg_distance = sum(valid_ranges) / len(valid_ranges)
                    print(f"   📡 LiDAR: {len(lidar_data)} puntos totales, {len(valid_ranges)} válidos")
                    print(f"   📡 Distancias: min={min_distance:.3f}m, max={max_distance:.3f}m, avg={avg_distance:.3f}m")
                    print(f"   📡 Obstáculo más cercano: {robot.get_lidar_closest_obstacle():.3f}m")
                else:
                    print(f"   📡 LiDAR: {len(lidar_data)} puntos, ninguno válido")
            
            # TEST: Información técnica del LiDAR
            print(f"   📊 LiDAR técnico:")
            print(f"      Rango: {robot.get_lidar_min_range():.2f} - {robot.get_lidar_max_range():.2f}m")
            print(f"      FOV: {math.degrees(robot.get_lidar_fov()):.1f}°")
            print(f"      Puntos: {robot.get_lidar_range_count()}")
        
        # Información básica cada 10 iteraciones
        elif iteration % 10 == 0:
            distance_to_center = math.sqrt(current_x**2 + current_y**2)
            wp_idx = controller.current_waypoint_index
            print(f"📍 Iter {iteration}: pos=({current_x:.2f},{current_y:.2f}) WP={wp_idx} dist_centro={distance_to_center:.2f}m")
        
        # Verificar si se completó la ruta
        if controller.is_route_completed():
            print("🎯 ¡Ruta circular completada! Reiniciando...")
            controller.reset()  # Reiniciar para repetir la ruta circular
    
    robot.cleanup()
    print("✅ Test de sensores con ruta circular finalizado")


if __name__ == "__main__":
    main()
