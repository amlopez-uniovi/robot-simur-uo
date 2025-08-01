# Controlador de navegación aleatoria para el robot e-puck
# Utiliza la clase EPuck y RandomNavigationController

import sys
import os
import math

# Agregar el directorio del paquete al path
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', '..'))

from robot_simur_uo import EPuck, RandomNavigationController


def main():
    """Función principal del controlador de navegación aleatoria para EPuck."""
    print("🤖 Iniciando controlador de navegación aleatoria para EPuck")
    print("=" * 60)
    
    # Crear instancia del robot e-puck
    robot = EPuck()
    
    # Crear controlador de navegación aleatoria
    # Espacio de trabajo más pequeño para el EPuck (robot más lento)
    controller = RandomNavigationController(
        workspace_bounds=(-1.9, 1.9, -1.9, 1.9),  # Espacio de 3x3 metros
        linear_gain=0.8,     # Ganancia más conservadora para EPuck
        steering_gain=1.5,   # Buena respuesta de giro
        goal_tolerance=0.1,  # Tolerancia de 10cm
        sensor_report_interval=50  # Reporte cada 50 iteraciones
    )
    
    print(f"📍 Espacio de trabajo: {controller.workspace_bounds}")
    print(f"🎯 Tolerancia objetivo: {controller.goal_tolerance}m")
    print(f"📊 Reporte sensores cada: {controller.sensor_report_interval} iteraciones")
    print("🚀 Comenzando navegación aleatoria...\n")
    
    iteration_count = 0
    
    try:
        # Bucle principal de control
        while robot.step() != -1:
            iteration_count += 1
            
            # Obtener posición y orientación actuales
            gps_position = robot.get_gps_position()
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
            
            # Mostrar progreso básico cada 100 iteraciones
            if iteration_count % 100 == 0:
                stats = controller.get_statistics()
                if controller.current_goal:
                    goal_distance = math.sqrt(
                        (controller.current_goal[0] - current_x)**2 + 
                        (controller.current_goal[1] - current_y)**2
                    )
                    print(f"Iteración {iteration_count}: Pos=({current_x:.2f}, {current_y:.2f}), "
                          f"Objetivo=({controller.current_goal[0]:.2f}, {controller.current_goal[1]:.2f}), "
                          f"Distancia={goal_distance:.2f}m, Objetivos={stats['goals_reached']}")
            
            # Detección simple de obstáculos para evitación
            if hasattr(robot, 'obstacle_detected') and robot.obstacle_detected():
                # Si hay obstáculo, reducir velocidad y aumentar giro
                drive_speed *= 0.3
                if drive_speed > 0:
                    steering_speed += 0.5  # Girar más para evitar obstáculo
                robot.set_drive_command(drive_speed, steering_speed)
    
    except KeyboardInterrupt:
        print("\n🛑 Deteniendo controlador por solicitud del usuario...")
    
    except Exception as e:
        print(f"\n❌ Error en el controlador: {e}")
    
    finally:
        # Detener robot y mostrar estadísticas finales
        robot.stop()
        
        stats = controller.get_statistics()
        print(f"\n📈 ESTADÍSTICAS FINALES:")
        print(f"   Iteraciones ejecutadas: {stats['iteration_count']}")
        print(f"   Objetivos alcanzados: {stats['goals_reached']}")
        print(f"   Objetivo actual: {stats['current_goal']}")
        print(f"   Objetivos recientes: {len(stats['recent_goals'])}")
        
        # Limpiar recursos
        robot.cleanup()
        print("🔄 Recursos liberados correctamente")
        print("✅ Controlador finalizado")


if __name__ == "__main__":
    main()
