"""
Ejemplo de uso del controlador Bug0NavigationController.

Este ejemplo demuestra cómo usar el algoritmo Bug0 para navegación con
evitación de obstáculos, combinando:
1. Navegación directa hacia el objetivo
2. Seguimiento de contorno cuando encuentra obstáculos  
3. Optimización de ruta cuando mejora la distancia al objetivo
"""

import math
import time
from robot_simur_uo import Bug0NavigationController, Bug0Mode


def simulate_obstacle_sensors(current_x: float, current_y: float, obstacles: list) -> list:
    """
    Simula sensores de obstáculos basados en obstáculos predefinidos.
    
    Args:
        current_x: Posición X actual del robot
        current_y: Posición Y actual del robot
        obstacles: Lista de obstáculos [(x, y, radio), ...]
        
    Returns:
        Lista de 8 distancias de sensores [front, front_right, right, etc.]
    """
    sensor_angles = [0, math.pi/4, math.pi/2, 3*math.pi/4, 
                    math.pi, 5*math.pi/4, 3*math.pi/2, 7*math.pi/4]
    sensor_range = 2.0
    sensor_distances = []
    
    for angle in sensor_angles:
        min_distance = sensor_range
        
        # Verificar cada obstáculo
        for obs_x, obs_y, obs_radius in obstacles:
            # Calcular distancia del robot al obstáculo
            dx = obs_x - current_x
            dy = obs_y - current_y
            distance_to_obstacle = math.sqrt(dx**2 + dy**2)
            
            # Calcular ángulo hacia el obstáculo
            angle_to_obstacle = math.atan2(dy, dx)
            
            # Verificar si el obstáculo está en la dirección del sensor
            angle_diff = abs(angle - angle_to_obstacle)
            if angle_diff > math.pi:
                angle_diff = 2 * math.pi - angle_diff
            
            # Si está en el cono del sensor y es más cercano
            if angle_diff < math.pi/8 and distance_to_obstacle < min_distance:
                # Restar el radio del obstáculo para obtener distancia al borde
                sensor_distance = max(0.1, distance_to_obstacle - obs_radius)
                min_distance = sensor_distance
        
        sensor_distances.append(min_distance)
    
    return sensor_distances


def main():
    """Ejemplo principal de navegación Bug0."""
    print("🐛 EJEMPLO DE NAVEGACIÓN BUG0")
    print("=" * 60)
    
    # Crear controlador Bug0
    controller = Bug0NavigationController(
        linear_gain=1.2,
        steering_gain=2.5,
        obstacle_threshold=0.6,
        wall_follow_distance=0.4,
        wall_follow_gain=1.5
    )
    
    # Definir objetivo
    goal_x, goal_y = 8.0, 6.0
    controller.set_target(goal_x, goal_y, tol=0.3)
    
    # Definir obstáculos en el entorno [(x, y, radio), ...]
    obstacles = [
        (3.0, 2.0, 0.8),   # Obstáculo 1
        (5.0, 4.0, 1.0),   # Obstáculo 2
        (7.0, 2.5, 0.6),   # Obstáculo 3
    ]
    
    print(f"\n🎯 Objetivo: ({goal_x}, {goal_y})")
    print(f"🚧 Obstáculos definidos: {len(obstacles)}")
    for i, (ox, oy, radius) in enumerate(obstacles):
        print(f"   Obstáculo {i+1}: centro=({ox}, {oy}), radio={radius}m")
    
    # Estado inicial del robot
    current_x, current_y, current_angle = 0.0, 0.0, 0.0
    dt = 0.2  # Paso de tiempo
    max_steps = 50
    
    print(f"\n🚀 INICIANDO SIMULACIÓN DE NAVEGACIÓN")
    print(f"   Posición inicial: ({current_x}, {current_y})")
    print(f"   Pasos máximos: {max_steps}")
    
    path = [(current_x, current_y)]  # Guardar trayectoria
    
    for step in range(max_steps):
        # Simular sensores de obstáculos
        obstacle_sensors = simulate_obstacle_sensors(current_x, current_y, obstacles)
        
        # Calcular comandos de control
        drive_speed, steering_speed = controller.calculate_control_commands(
            current_x, current_y, current_angle, obstacle_sensors
        )
        
        # Obtener estado del algoritmo
        status = controller.get_bug0_status()
        distance_to_goal = controller._distance_to_goal(current_x, current_y)
        
        # Mostrar progreso cada 5 pasos
        if step % 5 == 0 or status['mode'] != 'go_to_goal':
            print(f"\n📍 Paso {step + 1}:")
            print(f"   Posición: ({current_x:.2f}, {current_y:.2f})")
            print(f"   Modo: {status['mode']}")
            print(f"   Distancia al objetivo: {distance_to_goal:.2f}m")
            print(f"   Comandos: vel={drive_speed:.2f}, giro={steering_speed:.2f}")
            
            # Mostrar sensores si hay obstáculos cerca
            min_sensor = min(obstacle_sensors)
            if min_sensor < 1.0:
                print(f"   Sensor mínimo: {min_sensor:.2f}m")
        
        # Verificar si llegamos al objetivo
        if controller.is_target_reached(current_x, current_y):
            print(f"\n🎯 ¡OBJETIVO ALCANZADO en {step + 1} pasos!")
            break
        
        # Aplicar cinemática simple del robot
        # Limitar velocidades para simulación realista
        drive_speed = max(-2.0, min(2.0, drive_speed))
        steering_speed = max(-3.0, min(3.0, steering_speed))
        
        # Actualizar posición
        current_x += drive_speed * math.cos(current_angle) * dt
        current_y += drive_speed * math.sin(current_angle) * dt
        current_angle += steering_speed * dt
        
        # Normalizar ángulo
        current_angle = math.atan2(math.sin(current_angle), math.cos(current_angle))
        
        # Guardar posición en la trayectoria
        path.append((current_x, current_y))
        
        # Pequeña pausa para visualización
        time.sleep(0.1)
    
    # Mostrar resultados finales
    final_status = controller.get_bug0_status()
    final_distance = controller._distance_to_goal(current_x, current_y)
    
    print(f"\n📊 RESULTADOS FINALES:")
    print(f"   Posición final: ({current_x:.2f}, {current_y:.2f})")
    print(f"   Distancia final al objetivo: {final_distance:.2f}m")
    print(f"   Objetivo alcanzado: {'Sí' if final_distance < 0.3 else 'No'}")
    print(f"   Pasos ejecutados: {len(path)}")
    
    print(f"\n📈 ESTADÍSTICAS DEL ALGORITMO BUG0:")
    print(f"   Modo final: {final_status['mode']}")
    print(f"   Detecciones de obstáculos: {final_status['obstacle_detections']}")
    print(f"   Tiempo siguiendo pared: {final_status['wall_following_time']}")
    print(f"   Cambios de modo: {final_status['mode_switches']}")
    print(f"   Mejor distancia alcanzada: {final_status['best_distance_to_goal']:.2f}m")
    print(f"   Distancia inicial: {final_status['initial_goal_distance']:.2f}m")
    
    # Mostrar algunos puntos de la trayectoria
    print(f"\n🛤️  TRAYECTORIA (muestra cada 5 puntos):")
    for i in range(0, len(path), 5):
        x, y = path[i]
        print(f"   Punto {i}: ({x:.2f}, {y:.2f})")
    
    # Calcular distancia total recorrida
    total_distance = 0.0
    for i in range(1, len(path)):
        dx = path[i][0] - path[i-1][0]
        dy = path[i][1] - path[i-1][1]
        total_distance += math.sqrt(dx**2 + dy**2)
    
    print(f"\n📏 MÉTRICAS DE EFICIENCIA:")
    print(f"   Distancia total recorrida: {total_distance:.2f}m")
    print(f"   Distancia directa al objetivo: {math.sqrt(goal_x**2 + goal_y**2):.2f}m")
    print(f"   Factor de eficiencia: {(math.sqrt(goal_x**2 + goal_y**2) / total_distance * 100):.1f}%")
    
    print(f"\n✅ Simulación Bug0 completada!")


if __name__ == "__main__":
    main()
