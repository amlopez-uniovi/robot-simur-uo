# Controlador de navegación por waypoints para el robot RosBot
# Utiliza la clase RosBot y WaypointNavigationController

import math
from robot_simur_uo import RosBot, WaypointNavigationController


def main():
    """Función principal del controlador de navegación por waypoints para RosBot."""
    print("🤖 Iniciando controlador de navegación por waypoints para RosBot")
    print("=" * 70)
    
    # Crear instancia del robot RosBot
    robot = RosBot()
    
    # Definir lista de waypoints (puntos a seguir)
    waypoints = [
        (0.0, 0.0),    # Punto de inicio
        (1.0, 0.0),    # Derecha
        (1.0, 1.0),    # Arriba-derecha
        (0.0, 1.0),    # Arriba-izquierda
        (-1.0, 1.0),   # Más arriba-izquierda
        (-1.0, 0.0),   # Izquierda
        (-1.0, -1.0),  # Abajo-izquierda
        (0.0, -1.0),   # Abajo-centro
        (1.0, -1.0),   # Abajo-derecha
    ]
    
    # Crear controlador de navegación por waypoints
    # Configuración conservadora para RosBot
    controller = WaypointNavigationController(
        waypoints=waypoints,
        goal_tolerance=0.2,      # Tolerancia de 20cm (apropiada para RosBot)
        linear_gain=0.8,         # Ganancia lineal conservadora
        steering_gain=1.5,       # Ganancia de dirección
        max_linear_speed=0.1,    # Velocidad máxima lineal limitada para RosBot
        max_angular_speed=0.5,   # Velocidad angular limitada para RosBot
        cycle_waypoints=True     # Repetir la ruta indefinidamente
    )
    
    iteration_count = 0
    
    try:
        # Bucle principal de control
        while robot.step() != -1:
            iteration_count += 1
            
            # Obtener posición GPS (método común)
            gps_position = robot.get_gps_position()
            
            # Obtener orientación de la brújula (método común)
            compass_direction, compass_angle = robot.get_compass_orientation()
            
            current_x = gps_position[0]
            current_y = gps_position[1]
            current_angle = compass_angle
            
            # Actualizar controlador y obtener comandos
            drive_speed, steering_speed = controller.update(
                robot, current_x, current_y, current_angle
            )
            
            # Aplicar comandos al robot
            robot.set_drive_command(drive_speed, steering_speed)
            
            # Mostrar progreso cada 100 iteraciones
            if iteration_count % 100 == 0:
                # Obtener información de progreso
                progress = controller.get_progress_info()
                stats = controller.get_statistics()
                
                print(f"\n📍 PROGRESO - Iteración {iteration_count}")
                print(f"   Posición actual: ({current_x:.2f}, {current_y:.2f})")
                print(f"   Waypoint actual: {progress['current_waypoint_index'] + 1}/{progress['total_waypoints']}")
                
                if progress['current_target']:
                    target_x, target_y = progress['current_target']
                    distance_to_target = math.sqrt(
                        (target_x - current_x)**2 + (target_y - current_y)**2
                    )
                    print(f"   Objetivo: ({target_x:.2f}, {target_y:.2f})")
                    print(f"   Distancia al objetivo: {distance_to_target:.2f}m")
                
                print(f"   Waypoints alcanzados: {progress['waypoints_reached']}")
                print(f"   Ciclos completados: {progress['total_cycles_completed']}")
                print(f"   Progreso del ciclo: {progress['progress_percent']:.1f}%")
                
                # Log detallado de dispositivos cada 100 iteraciones
                print(f"\n📊 LOG DETALLADO DE DISPOSITIVOS - Iteración {iteration_count}")
                robot.log_devices(to_terminal=True)
                print("=" * 70)
            
            # Detección simple de obstáculos para evitación
            if hasattr(robot, 'obstacle_detected') and robot.obstacle_detected():
                # Si hay obstáculo, reducir velocidad y aumentar giro
                drive_speed *= 0.3
                if drive_speed > 0:
                    steering_speed += 0.5  # Girar más para evitar obstáculo
                robot.set_drive_command(drive_speed, steering_speed)
                
                if iteration_count % 50 == 0:  # Mensaje menos frecuente para obstáculos
                    print(f"⚠️  Obstáculo detectado - Maniobra evasiva activada")
    
    except KeyboardInterrupt:
        print("\n🛑 Deteniendo controlador por solicitud del usuario...")
    
    except Exception as e:
        print(f"\n❌ Error en el controlador: {e}")
    
    finally:
        # Detener robot y mostrar estadísticas finales
        robot.stop()
        
        # Mostrar estadísticas finales completas
        final_stats = controller.get_statistics()
        final_progress = controller.get_progress_info()
        
        print(f"\n📈 ESTADÍSTICAS FINALES - WAYPOINT NAVIGATION:")
        print(f"   Iteraciones ejecutadas: {final_stats['iteration_count']}")
        print(f"   Waypoints alcanzados: {final_progress['waypoints_reached']}")
        print(f"   Ciclos completados: {final_progress['total_cycles_completed']}")
        print(f"   Total de waypoints: {final_progress['total_waypoints']}")
        print(f"   Waypoint actual: {final_progress['current_waypoint_index'] + 1}")
        print(f"   Progreso del ciclo: {final_progress['progress_percent']:.1f}%")
        print(f"   Ruta completada: {'Sí' if final_progress['is_route_completed'] else 'No'}")
        print(f"   Modo cíclico: {'Sí' if final_progress['cycle_mode'] else 'No'}")
        
        # Limpiar recursos
        robot.cleanup()
        print("🔄 Recursos liberados correctamente")
        print("✅ Controlador de waypoints finalizado")


if __name__ == "__main__":
    main()
