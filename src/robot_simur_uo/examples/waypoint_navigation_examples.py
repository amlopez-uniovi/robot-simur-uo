"""
Ejemplos de uso del WaypointNavigationController.

Este módulo contiene diferentes ejemplos que muestran cómo crear y configurar
controladores de navegación por waypoints sin necesidad de una instancia real del robot.
"""

import math
from typing import List, Tuple
from robot_simur_uo.controllers.waypoint_navigation import WaypointNavigationController


def demo_custom_waypoints():
    """Demostración con waypoints personalizados."""
    print("\n🎯 DEMO 1: Waypoints personalizados")
    
    # Waypoints que forman una estrella
    waypoints = [
        (0.0, 0.0),    # Centro
        (1.0, 0.0),    # Derecha
        (-0.5, 0.8),   # Arriba-izquierda
        (0.5, -0.8),   # Abajo-derecha
        (-1.0, 0.0),   # Izquierda
        (0.5, 0.8),    # Arriba-derecha
        (-0.5, -0.8),  # Abajo-izquierda
    ]
    
    controller = WaypointNavigationController(
        waypoints=waypoints,
        goal_tolerance=0.15,
        cycle_waypoints=False  # Solo una vez
    )
    
    print(f"Waypoints definidos: {len(waypoints)}")
    for i, (x, y) in enumerate(waypoints):
        print(f"  {i+1}: ({x:.1f}, {y:.1f})")
    
    return controller


def demo_square_route():
    """Demostración con ruta cuadrada predefinida."""
    print("\n🔷 DEMO 2: Ruta cuadrada")
    
    controller = WaypointNavigationController.create_square_route(
        center_x=0.0,
        center_y=0.0,
        size=2.0,
        goal_tolerance=0.1,
        cycle_waypoints=True
    )
    
    print("Ruta cuadrada de 2x2 metros centrada en (0,0)")
    waypoints = controller.waypoints
    for i, (x, y) in enumerate(waypoints):
        print(f"  Esquina {i+1}: ({x:.1f}, {y:.1f})")
    
    return controller


def demo_circular_route():
    """Demostración con ruta circular predefinida."""
    print("\n⭕ DEMO 3: Ruta circular")
    
    controller = WaypointNavigationController.create_circular_route(
        center_x=0.0,
        center_y=0.0,
        radius=1.5,
        num_points=12,  # 12 puntos = 30° entre cada uno
        goal_tolerance=0.1,
        cycle_waypoints=True
    )
    
    print("Ruta circular de radio 1.5m con 12 puntos")
    waypoints = controller.waypoints
    for i, (x, y) in enumerate(waypoints):
        angle = math.atan2(y, x) * 180 / math.pi
        print(f"  Punto {i+1}: ({x:.1f}, {y:.1f}) - Ángulo: {angle:.0f}°")
    
    return controller


def demo_dynamic_waypoints():
    """Demostración de modificación dinámica de waypoints."""
    print("\n🔧 DEMO 4: Modificación dinámica")
    
    # Empezar con waypoints básicos
    initial_waypoints = [
        (0.0, 0.0),
        (1.0, 0.0),
        (1.0, 1.0),
    ]
    
    controller = WaypointNavigationController(
        waypoints=initial_waypoints,
        goal_tolerance=0.1,
        cycle_waypoints=True
    )
    
    print("Waypoints iniciales:")
    for i, (x, y) in enumerate(controller.waypoints):
        print(f"  {i+1}: ({x:.1f}, {y:.1f})")
    
    # Añadir waypoints dinámicamente
    controller.add_waypoint(0.0, 1.0)  # Completar el cuadrado
    controller.add_waypoint(0.5, 0.5, insert_at=1)  # Insertar en posición 1
    
    print("\nDespués de añadir waypoints:")
    for i, (x, y) in enumerate(controller.waypoints):
        print(f"  {i+1}: ({x:.1f}, {y:.1f})")
    
    return controller


def main():
    """Función principal de demostración."""
    print("🚀 DEMOS DEL WAYPOINTNAVIGATIONCONTROLLER")
    print("=" * 50)
    
    # Ejecutar las diferentes demostraciones (sin crear robot real)
    controllers = [
        demo_custom_waypoints(),
        demo_square_route(),
        demo_circular_route(),
        demo_dynamic_waypoints()
    ]
    
    print("\n📊 RESUMEN DE CONTROLADORES CREADOS:")
    for i, controller in enumerate(controllers, 1):
        progress = controller.get_progress_info()
        print(f"\nControlador {i}:")
        print(f"  - Waypoints: {progress['total_waypoints']}")
        print(f"  - Modo cíclico: {'Sí' if progress['cycle_mode'] else 'No'}")
        print(f"  - Tolerancia: {controller.goal_tolerance}m")
        print(f"  - Objetivo actual: {progress['current_target']}")
    
    print("\n" + "=" * 50)
    print("Para usar cualquiera de estos controladores en un robot:")
    print("1. Copia el código de creación del controlador")
    print("2. Úsalo en un bucle de control como en los otros ejemplos")
    print("3. Llama a controller.update() en cada iteración")


if __name__ == "__main__":
    main()
