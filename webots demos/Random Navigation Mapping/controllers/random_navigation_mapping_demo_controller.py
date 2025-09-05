"""
Controlador base para la demo de navegación aleatoria (EPuck y RosBot)
Estructura unificada: inicialización, bucle principal, sensores, control y logging
"""
from robot_simur_uo.controllers.random_navigation import RandomNavigationController
from robot_simur_uo.utils.grid_map import GridMap
from robot_simur_uo.utils.probabilistic_grid_map import ProbabilisticGridMap
from robot_simur_uo.utils.probabilistic_logodds_grid_map import ProbabilisticLogOddsGridMap
from robot_simur_uo.utils.coordinates import transform_points
import math

def run_random_navigation_mapping_demo(RobotClass):
    robot = RobotClass()
    #prob_grid_map = ProbabilisticGridMap((-2.2, -2.2), (2.2, 2.2), resolution=0.1, empty_value=0.5, occupancy_factor=0.7, free_factor=0.3)  # Mapa de 4.4x4.4 metros con resolución de 10cm
    prob_grid_map = ProbabilisticLogOddsGridMap((-2.2, -2.2), (2.2, 2.2), resolution=0.1, empty_value=0.0, occupancy_factor=1.95, free_factor=-1.1)  # Mapa de 4.4x4.4 metros con resolución de 10cm


    # Crear controlador de navegación aleatoria
    # Espacio de trabajo más pequeño para el robot (robot más lento)
    controller = RandomNavigationController(
        workspace_bounds=(-1.9, 1.9, -1.9, 1.9),  # Espacio de 3x3 metros
        linear_gain=0.8,     # Ganancia más conservadora para RosBot
        steering_gain=2.5,   # Buena respuesta de giro
        goal_tolerance=0.1  # Tolerancia de 10cm
    )
    
    iteration_count = 0
    fig_visualizacion = None
    # Bucle principal de control
    while robot.step() != -1:
        iteration_count += 1
        
        # Obtener medidas del lidar del robot
        #lidar_data = robot.get_lidar_manager().get_raw_data_with_angles()
        #print(f"Medidas LIDAR: {lidar_data}")

        # Actualizar mapa de ocupación usando puntos XY de obstáculos detectados
        obstacle_points, free_points = robot.get_lidar_manager().get_obstacle_points_xy()
        prob_grid_map.update(obstacle_points, free_points, robot.get_pose().to_tuple())

        pose = robot.get_pose().to_tuple()
        
#        celdas_ocupadas, celdas_libres = grid_map.get_occupied_free_cells_from_pose_obstacles(
#            pose, obstacle_points, free_points, points_in_robot_frame=True
#        )
        
#        grid_map.reset()  # Limpiar el mapa antes de actualizarlo

        # Actualizar el mapa de ocupación con las celdas ocupadas y libres
    #    for cell in celdas_ocupadas:
    #        grid_map.set_cell(*cell, -1.0)  # Marcar celda como ocupada
    #    for cell in celdas_libres:
    #        grid_map.set_cell(*cell, 1)  # Marcar celda como libre

    #    print(f"Iteración {iteration_count}")
    #    print(f"Pose actual del robot: {pose}")
    #    print(f"Puntos de obstáculos detectados (robot frame): {obstacle_points}")
    #    print(f"Puntos libres detectados (robot frame): {free_points}")
    #    print(f"Celdas ocupadas detectadas: {sorted(celdas_ocupadas, key=lambda x: x[0])}")
    #    print(f"Celdas libres detectadas: {sorted(celdas_libres, key=lambda x: x[0])}")
    #    print("-" * 40)
    
    
#        obstacle_points_tf = transform_points(obstacle_points, pose)
#        free_points_tf = transform_points(free_points, pose)

#        grid_map.reset()  # Limpiar el mapa antes de actualizarlo
        
        # Actualizar el mapa de ocupación con los puntos transformados
#        obstacle_points_map = [grid_map.world_to_map(x, y) for x, y in obstacle_points_tf]
        
        #for x, y in obstacle_points_map:
#       for x, y in celdas_ocupadas:
#           grid_map.set_cell(x, y, 1.0, world_coordinates=False)  # Marcar celda como ocupada

        #for x, y in obstacle_points_map:
#        for x, y in celdas_libres:
#            grid_map.set_cell(x, y, -1.0, world_coordinates=False)  # Marcar celda como ocupada

        # Pintar el mapa de ocupación (visualización simple en consola)
        fig_visualizacion = prob_grid_map.visualize(fig=fig_visualizacion)

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