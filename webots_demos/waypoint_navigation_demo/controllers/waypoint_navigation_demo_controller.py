
"""
Controlador base para la demo de navegación por waypoints (EPuck y RosBot).

Estructura unificada: inicialización, bucle principal, lectura de sensores y logging.

Funciones:
    run_waypoint_navigation_demo(RobotClass): Ejecuta la demo de navegación por waypoints para el robot especificado.

Ejemplo:
    from waypoint_navigation_demo_controller import run_waypoint_navigation_demo
    from robot_simur_uo.webots.epuck_robot import EPuck
    run_waypoint_navigation_demo(EPuck)
"""
import math
from robot_simur_uo import WaypointNavigationController, Waypoints


def run_waypoint_navigation_demo(RobotClass):
    robot = RobotClass()

    
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
    # Configuración conservadora para RosBot
    controller = WaypointNavigationController(
        waypoints=waypoints,  # Pasar la instancia de Waypoints
        goal_tolerance=0.2,        # Tolerancia de 20cm (apropiada para RosBot)
        linear_gain=0.8,           # Ganancia lineal conservadora
        steering_gain=1.5,         # Ganancia de dirección
        max_linear_speed=0.1,      # Velocidad máxima lineal limitada para RosBot
        max_angular_speed=0.5,     # Velocidad angular limitada para RosBot
        cycle_waypoints=True,      # Repetir la ruta indefinidamente
        lookahead_factor=0.25      # Factor de lookahead optimizado para RosBot
    )
    
    iteration_count = 0
    
    try:
        # Bucle principal de control
        while robot.step() != -1:
            iteration_count += 1
            
            pose = robot.get_pose()
            current_x, current_y, current_angle = pose.x, pose.y, pose.theta

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
