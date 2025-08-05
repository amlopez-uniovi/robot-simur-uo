# Controlador de test de sensores para RosBot - Giro 360° sobre sí mismo
# Gira 360° usando turn_left() mientras testea todos los sensores

import math
from robot_simur_uo import RosBot


def main():
    """Controlador de test de sensores girando 360° sobre sí mismo para RosBot."""
    print("🔬 RosBot - Test de Sensores con Giro 360°")
    
    # Robot RosBot
    robot = RosBot()
    
    # Verificar LidarManager
    lidar_manager = robot.get_lidar_manager()
    if robot.has_lidar_manager():
        print("📡 LidarManager disponible en RosBot")
        
        # Mostrar configuración básica sin datos (evitar crash inicial)
        config = lidar_manager.get_configuration_info()
        print(f"📊 Configuración LiDAR:")
        print(f"   Dispositivo: {config['device_name']}")
        print(f"   Puntos: {config['range_count']}")
        print(f"   Rango: {config['min_range']:.3f}m - {config['max_range']:.3f}m")
        print(f"   FOV: {math.degrees(config['fov']):.1f}°")
    else:
        print("⚠️ LidarManager no disponible, continuando sin él...")
    
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
        
        # Inicializar ángulo de referencia
        if start_angle is None:
            start_angle = current_angle
            print(f"📍 Posición inicial: ({current_x:.3f}, {current_y:.3f})")
            print(f"🧭 Ángulo inicial: {math.degrees(current_angle):.1f}°")
        
        # Calcular rotación total desde el inicio
        angle_diff = current_angle - start_angle
        # Normalizar la diferencia de ángulo
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        # Acumular rotación (considerando cambios de signo en la normalización)
        if iteration > 1:
            prev_angle_diff = getattr(main, 'prev_angle_diff', 0)
            delta = angle_diff - prev_angle_diff
            # Detectar cruce de la discontinuidad
            if delta > math.pi:
                delta -= 2 * math.pi
            elif delta < -math.pi:
                delta += 2 * math.pi
            total_rotation += abs(delta)
        
        main.prev_angle_diff = angle_diff
        
        # Girar a la izquierda continuamente
        robot.turn_left()

        # Log de dispositivos normales
        robot.log_devices()
        
        # Mostrar datos LiDAR con ángulos en cada iteración
        if robot.has_lidar_manager():
            try:
                raw_data_with_angles = lidar_manager.get_raw_data_with_angles()
                if raw_data_with_angles:
                    print("Lidar ángulos-distancia:")
                    # Mostrar primeros 10 puntos para no saturar la salida
                    for i, (distance, angle) in enumerate(raw_data_with_angles[:10]):
                        if math.isinf(distance):
                            print(f"  [{i}]: ({math.degrees(angle):6.1f}°, inf)")
                        else:
                            print(f"  [{i}]: ({math.degrees(angle):6.1f}°, {distance:6.3f}m)")
                    if len(raw_data_with_angles) > 10:
                        print(f"  ... y {len(raw_data_with_angles) - 10} puntos más")
            except Exception as e:
                print(f"Error LiDAR: {e}")
        
        # Test específico de LidarManager cada 50 iteraciones
        if robot.has_lidar_manager() and iteration % 50 == 0:
            print(f"\n📡 === Test LidarManager RosBot - Iteración {iteration} ===")
            
            try:
                # Obtener datos crudos con ángulos directamente
                raw_data_with_angles = lidar_manager.get_raw_data_with_angles()
                print(f"📊 Datos LiDAR con ángulos: {len(raw_data_with_angles)} puntos")
                
                # Imprimir todos los pares (ángulo, distancia)
                if raw_data_with_angles:
                    print("   Todos los puntos (ángulo°, distancia):")
                    for i, (distance, angle) in enumerate(raw_data_with_angles):
                        if math.isinf(distance):
                            print(f"     [{i:3d}]: ({math.degrees(angle):6.1f}°, inf)")
                        else:
                            print(f"     [{i:3d}]: ({math.degrees(angle):6.1f}°, {distance:6.3f}m)")
                else:
                    print("   No hay datos LiDAR disponibles")
                
                # Obtener datos filtrados
                filtered_data = lidar_manager.get_filtered_data()
                print(f"🔍 Datos filtrados: {len(filtered_data)} puntos válidos")
                
                # Encontrar obstáculos cercanos (RosBot tiene mayor rango)
                obstacles = lidar_manager.find_obstacles(threshold=2.5)
                print(f"🚧 Obstáculos (<2.5m): {len(obstacles)} detectados")
                
                # Mostrar algunos obstáculos si los hay
                if obstacles:
                    print("   Primeros 5 obstáculos (índice, distancia, ángulo):")
                    for i, (idx, dist, angle) in enumerate(obstacles[:5]):
                        print(f"     #{idx}: {dist:.3f}m a {math.degrees(angle):.1f}°")
                
                # Estadísticas
                stats = lidar_manager.get_statistics()
                if stats:
                    print(f"📈 Estadísticas:")
                    print(f"   Válidos: {stats.get('valid_points', 0)}")
                    print(f"   Infinitos: {stats.get('infinite_points', 0)}")
                    print(f"   Ceros: {stats.get('zero_points', 0)}")
                    if 'avg_distance' in stats:
                        print(f"   Distancia promedio: {stats['avg_distance']:.3f}m")
                
            except Exception as e:
                print(f"⚠️ Error en test LiDAR: {e}")
            
            print("=" * 50)

        # Información básica cada 25 iteraciones
        if iteration % 25 == 0:
            print(f"🔄 Iter {iteration}: ángulo={math.degrees(current_angle):.1f}° rotación={math.degrees(total_rotation):.1f}°")
        
        # Verificar si completó una vuelta completa (360°)
        if total_rotation >= 2 * math.pi:
            print(f"\n🎯 ¡Giro completo de 360° completado!")
            print(f"   Iteraciones totales: {iteration}")
            print(f"   Rotación final: {math.degrees(total_rotation):.1f}°")
            print(f"   Posición final: ({current_x:.3f}, {current_y:.3f})")
            
            # Último test del LidarManager
            if robot.has_lidar_manager():
                print(f"\n📡 === Test Final LidarManager RosBot ===")
                try:
                    # Imprimir datos finales directamente
                    raw_data_with_angles = lidar_manager.get_raw_data_with_angles()
                    print(f"📊 Datos LiDAR finales: {len(raw_data_with_angles)} puntos")
                    
                    if raw_data_with_angles:
                        print("   Datos finales (ángulo°, distancia):")
                        for i, (distance, angle) in enumerate(raw_data_with_angles):
                            if math.isinf(distance):
                                print(f"     [{i:3d}]: ({math.degrees(angle):6.1f}°, inf)")
                            else:
                                print(f"     [{i:3d}]: ({math.degrees(angle):6.1f}°, {distance:6.3f}m)")
                    
                    # Mostrar configuración final
                    config = lidar_manager.get_configuration_info()
                    print(f"🔧 Configuración final:")
                    print(f"   Dispositivo: {config['device_name']}")
                    print(f"   Puntos: {config['range_count']}")
                    print(f"   FOV: {math.degrees(config['fov']):.1f}°")
                    
                except Exception as e:
                    print(f"⚠️ Error en test final LiDAR: {e}")
            
            # Detener el robot
            robot.stop()
            break  # Salir del bucle

    robot.cleanup()
    print("✅ Test de sensores con giro 360° y LidarManager finalizado")


if __name__ == "__main__":
    main()
