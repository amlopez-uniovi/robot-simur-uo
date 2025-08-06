"""
Controlador base para la demo de test de sensores (EPuck y RosBot)
Estructura unificada: inicialización, bucle principal, lectura de sensores y logging
"""

import math

def run_sensor_test_demo(RobotClass):
    robot = RobotClass()
    iteration = 0
    print(f"Iniciando demo de test de sensores para {RobotClass.__name__}...")

    
    iteration = 0
    start_angle = None
    total_rotation = 0.0
    prev_angle_diff = 0.0  # Inicializar diferencia de ángulo previa
    
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
        delta = angle_diff - prev_angle_diff
        # Detectar cruce de la discontinuidad
        if delta > math.pi:
            delta -= 2 * math.pi
        elif delta < -math.pi:
            delta += 2 * math.pi
        total_rotation += abs(delta)
        
        prev_angle_diff = angle_diff
        
        # Girar a la izquierda continuamente
        robot.turn_left()

        # Log completo de todos los dispositivos (incluye LiDAR con ángulos)
        robot.log_devices()

        # Verificar si completó una vuelta completa (360°)
        if total_rotation >= 2 * math.pi:
            print(f"\n🎯 ¡Giro completo de 360° completado!")
            print(f"   Iteraciones totales: {iteration}")
            print(f"   Rotación final: {math.degrees(total_rotation):.1f}°")
            print(f"   Posición final: ({current_x:.3f}, {current_y:.3f})")
            

            # Detener el robot
            robot.stop()
            break  # Salir del bucle        
        
        
    robot.cleanup()
