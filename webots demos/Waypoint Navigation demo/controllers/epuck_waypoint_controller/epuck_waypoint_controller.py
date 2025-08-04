# Controlador de navegación por waypoints para el robot e-puck
# Utiliza la clase EPuck y WaypointNavigationController

import math
from robot_simur_uo import EPuck, WaypointNavigationController, Waypoints


def main():
    """Función principal del controlador de navegación por waypoints para EPuck."""
    print("🤖 Iniciando controlador de navegación por waypoints para EPuck")
    print("=" * 70)
    
    # Crear instancia del robot e-puck
    robot = EPuck()
    
    # Crear waypoints usando la clase Waypoints
    waypoints = Waypoints()
    
    # Opción 1: Usar método para crear ruta predefinida (descomentar la que prefieras)
    # waypoints.create_square_route(center_x=0, center_y=0, size=2.0)
    # waypoints.create_circular_route(center_x=0, center_y=0, radius=1.0, num_points=8)
    # waypoints.create_rectangular_route(center_x=0, center_y=0, width=2.0, height=1.0)

    # Opción 2: Definir waypoints personalizados
    custom_waypoints = [
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
    waypoints.set_waypoints(custom_waypoints)
    
    print(f"📍 Waypoints creados:")
    waypoints.print_waypoints()
    
    # Crear controlador de navegación por waypoints
    # Configuración conservadora para EPuck
    controller = WaypointNavigationController(
        waypoints=waypoints,  # Pasar la instancia de Waypoints
        goal_tolerance=0.2,        # Tolerancia de 20cm (apropiada para EPuck)
        linear_gain=0.8,           # Ganancia lineal conservadora
        steering_gain=1.5,         # Ganancia de dirección
        max_linear_speed=0.1,      # Velocidad máxima lineal limitada para EPuck
        max_angular_speed=0.5,     # Velocidad angular limitada para EPuck
        cycle_waypoints=True,      # Repetir la ruta indefinidamente
        lookahead_factor=0.2       # Factor de lookahead optimizado para EPuck (más pequeño)
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
