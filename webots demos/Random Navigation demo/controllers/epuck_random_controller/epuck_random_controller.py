# Controlador de navegación aleatoria para el robot e-puck
# Utiliza la clase EPuck y RandomNavigationController

import math
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
        goal_tolerance=0.1  # Tolerancia de 10cm
    )
    
    iteration_count = 0
    
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
        
        # Mostrar progreso básico cada 100 iteraciones
        if iteration_count % 100 == 0:

            #######
            # Mostrar progreso básico cada 100 iteraciones
            stats = controller.get_statistics()
            if controller.current_goal:
                goal_distance = math.sqrt(
                    (controller.current_goal[0] - current_x)**2 + 
                    (controller.current_goal[1] - current_y)**2
                )
                print(f"Iteración {iteration_count}: Pos=({current_x:.2f}, {current_y:.2f}), "
                        f"Objetivo=({controller.current_goal[0]:.2f}, {controller.current_goal[1]:.2f}), "
                        f"Distancia={goal_distance:.2f}m, Objetivos={stats['goals_reached']}")
            
            print("=" * 60)                
            
            
            
            ######
            # Sí, este bloque de código puede considerarse redundante si la detección y evasión de obstáculos ya está implementada dentro del propio controlador o en otro lugar del código. 
            # Si no necesitas la lógica de evasión de obstáculos aquí, puedes eliminarlo junto con los bloques except y finally relacionados con la gestión de errores y estadísticas finales.


if __name__ == "__main__":
    main()
