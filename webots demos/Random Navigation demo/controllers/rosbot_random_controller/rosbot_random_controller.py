# Controlador de navegación aleatoria para el robot RosBot
# Utiliza la clase RosBot y RandomNavigationController

import math
from robot_simur_uo import RosBot, RandomNavigationController

def main():
    """Función principal del controlador de navegación aleatoria para RosBot."""
    print("🤖 Iniciando controlador de navegación aleatoria para RosBot")
    print("=" * 60)
    
    # Crear instancia del robot RosBot
    robot = RosBot()
    
    # Crear controlador de navegación aleatoria
    # Espacio de trabajo más amplio para el RosBot (robot más rápido y potente)
    controller = RandomNavigationController(
        workspace_bounds=(-2.5, 2.5, -2.5, 2.5),  # Espacio de 5x5 metros
        linear_gain=1.2,     # Ganancia más agresiva para RosBot
        steering_gain=2.0,   # Buena respuesta de giro
        goal_tolerance=0.15  # Tolerancia de 15cm
    )
    
    iteration_count = 0
    
    try:
        # Bucle principal de control
        while robot.step() != -1:
            iteration_count += 1
            
            # Obtener posición y orientación actuales usando el controlador de navegación
            current_x, current_y, current_angle = controller.get_robot_state(robot)
            
            # Actualizar controlador y obtener comandos
            drive_speed, steering_speed = controller.update(
                robot, current_x, current_y, current_angle
            )
            
            # Aplicar comandos al robot
            robot.set_drive_command(drive_speed, steering_speed)
            
            # Mostrar progreso básico cada 75 iteraciones (más frecuente para RosBot)
            if iteration_count % 75 == 0:

                #######
                # Mostrar progreso básico cada 75 iteraciones
                stats = controller.get_statistics()
                if controller.current_goal:
                    goal_distance = math.sqrt(
                        (controller.current_goal[0] - current_x)**2 + 
                        (controller.current_goal[1] - current_y)**2
                    )
                    print(f"Iteración {iteration_count}: Pos=({current_x:.2f}, {current_y:.2f}), "
                          f"Objetivo=({controller.current_goal[0]:.2f}, {controller.current_goal[1]:.2f}), "
                          f"Distancia={goal_distance:.2f}m, Objetivos={stats['goals_reached']}")
                
                # Añadir log detallado de dispositivos RosBot cada 75 iteraciones
                print(f"\n📊 LOG DETALLADO DE DISPOSITIVOS ROSBOT - Iteración {iteration_count}")
                robot.log_devices(to_terminal=True)
                print("=" * 60)                
                
                ######

            # Detección avanzada de obstáculos usando sensores específicos del RosBot
            try:
                distance_values = robot.get_distance_sensor_values()
                obstacle_threshold = 0.6  # 60cm de distancia mínima para RosBot
                
                # RosBot tiene 4 sensores: fl_range, rl_range, fr_range, rr_range
                front_left_distance = distance_values[0]   # fl_range
                rear_left_distance = distance_values[1]    # rl_range  
                front_right_distance = distance_values[2]  # fr_range
                rear_right_distance = distance_values[3]   # rr_range
                
                # Verificar obstáculos frontales principalmente
                obstacle_detected = (front_left_distance < obstacle_threshold or 
                                   front_right_distance < obstacle_threshold)
                
                if obstacle_detected:
                    print(f"⚠️  Obstáculo detectado: FL={front_left_distance:.2f}m, FR={front_right_distance:.2f}m")
                    
                    # Reducir velocidad y decidir dirección de giro
                    drive_speed *= 0.3
                    if drive_speed > 0:
                        # Decidir dirección basada en qué sensor detecta el obstáculo
                        if front_left_distance < front_right_distance:
                            steering_speed += 0.7  # Girar a la derecha
                        else:
                            steering_speed -= 0.7  # Girar a la izquierda
                    
                    robot.set_drive_command(drive_speed, steering_speed)
                    
            except Exception as e:
                # Si no hay sensores de distancia disponibles, usar método básico
                if hasattr(robot, 'obstacle_detected') and robot.obstacle_detected():
                    drive_speed *= 0.4
                    if drive_speed > 0:
                        steering_speed += 0.6
                    robot.set_drive_command(drive_speed, steering_speed)
    
    except KeyboardInterrupt:
        print("\n🛑 Deteniendo controlador por solicitud del usuario...")
    
    except Exception as e:
        print(f"\n❌ Error en el controlador: {e}")
    
    finally:
        # Detener robot y mostrar estadísticas finales
        robot.stop()
        
        stats = controller.get_statistics()
        print(f"\n📈 ESTADÍSTICAS FINALES ROSBOT:")
        print(f"   Iteraciones ejecutadas: {stats['iteration_count']}")
        print(f"   Objetivos alcanzados: {stats['goals_reached']}")
        print(f"   Objetivo actual: {stats['current_goal']}")
        print(f"   Objetivos recientes: {len(stats['recent_goals'])}")
        
        # Limpiar recursos
        robot.cleanup()
        print("🔄 Recursos liberados correctamente")
        print("✅ Controlador RosBot finalizado")


if __name__ == "__main__":
    main()
