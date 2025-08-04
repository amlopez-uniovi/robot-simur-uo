"""
Ejemplos de creación de waypoints fuera de la clase WaypointNavigationController.

Este archivo demuestra diferentes formas de crear waypoints usando la clase Waypoints
y cómo pasarlos al WaypointNavigationController.
"""

from ..utils.waypoints import Waypoints
from ..controllers.waypoint_navigation import WaypointNavigationController


def example_square_route():
    """Ejemplo de creación de ruta cuadrada."""
    print("🔷 EJEMPLO: Ruta cuadrada")
    print("-" * 30)
    
    # Crear waypoints fuera del controlador
    waypoints = Waypoints()
    waypoints.create_square_route(center_x=2, center_y=2, size=3.0)
    
    print("Waypoints creados:")
    waypoints.print_waypoints()
    
    # Pasar waypoints al controlador
    controller = WaypointNavigationController(
        waypoints=waypoints,
        lookahead_factor=0.3
    )
    
    print(f"✅ Controlador creado con {len(controller.waypoints)} waypoints")
    return controller


def example_circular_route():
    """Ejemplo de creación de ruta circular."""
    print("\n⭕ EJEMPLO: Ruta circular")
    print("-" * 30)
    
    # Crear waypoints fuera del controlador
    waypoints = Waypoints()
    waypoints.create_circular_route(center_x=0, center_y=0, radius=1.5, num_points=10)
    
    print("Waypoints creados:")
    waypoints.print_waypoints()
    
    # Pasar waypoints al controlador
    controller = WaypointNavigationController(
        waypoints=waypoints,
        lookahead_factor=0.25
    )
    
    print(f"✅ Controlador creado con {len(controller.waypoints)} waypoints")
    return controller


def example_custom_waypoints():
    """Ejemplo de waypoints personalizados."""
    print("\n📍 EJEMPLO: Waypoints personalizados")
    print("-" * 30)
    
    # Crear waypoints fuera del controlador
    waypoints = Waypoints()
    
    # Definir waypoints personalizados
    custom_points = [
        (0.0, 0.0),   # Origen
        (2.0, 0.0),   # Este
        (2.0, 2.0),   # Noreste
        (0.0, 2.0),   # Norte
        (-2.0, 2.0),  # Noroeste
        (-2.0, 0.0),  # Oeste
        (-2.0, -2.0), # Suroeste
        (0.0, -2.0),  # Sur
        (2.0, -2.0),  # Sureste
    ]
    
    waypoints.set_waypoints(custom_points)
    
    print("Waypoints creados:")
    waypoints.print_waypoints()
    
    # Pasar waypoints al controlador
    controller = WaypointNavigationController(
        waypoints=waypoints,
        lookahead_factor=0.2,
        goal_tolerance=0.15
    )
    
    print(f"✅ Controlador creado con {len(controller.waypoints)} waypoints")
    return controller


def example_line_route():
    """Ejemplo de ruta lineal."""
    print("\n📏 EJEMPLO: Ruta lineal")
    print("-" * 30)
    
    # Crear waypoints fuera del controlador
    waypoints = Waypoints()
    waypoints.create_line_route(start_x=0, start_y=0, end_x=5, end_y=3, num_points=6)
    
    print("Waypoints creados:")
    waypoints.print_waypoints()
    
    # Pasar waypoints al controlador
    controller = WaypointNavigationController(
        waypoints=waypoints,
        lookahead_factor=0.4
    )
    
    print(f"✅ Controlador creado con {len(controller.waypoints)} waypoints")
    return controller


def example_rectangular_route():
    """Ejemplo de ruta rectangular."""
    print("\n▭ EJEMPLO: Ruta rectangular")
    print("-" * 30)
    
    # Crear waypoints fuera del controlador
    waypoints = Waypoints()
    waypoints.create_rectangular_route(center_x=1, center_y=1, width=4.0, height=2.0)
    
    print("Waypoints creados:")
    waypoints.print_waypoints()
    
    # Pasar waypoints al controlador
    controller = WaypointNavigationController(
        waypoints=waypoints,
        lookahead_factor=0.3
    )
    
    print(f"✅ Controlador creado con {len(controller.waypoints)} waypoints")
    return controller


def example_combined_waypoints():
    """Ejemplo de combinación de diferentes tipos de waypoints."""
    print("\n🔄 EJEMPLO: Waypoints combinados")
    print("-" * 30)
    
    # Crear waypoints fuera del controlador
    waypoints = Waypoints()
    
    # Crear una ruta inicial
    waypoints.create_square_route(center_x=0, center_y=0, size=2.0)
    
    # Añadir waypoints adicionales
    additional_points = [
        (2.0, 2.0),   # Punto extra 1
        (3.0, 1.0),   # Punto extra 2
        (2.0, 0.0),   # Punto extra 3
    ]
    
    for point in additional_points:
        waypoints.add_waypoint(point[0], point[1])
    
    print("Waypoints creados:")
    waypoints.print_waypoints()
    
    # Pasar waypoints al controlador
    controller = WaypointNavigationController(
        waypoints=waypoints,
        lookahead_factor=0.25
    )
    
    print(f"✅ Controlador creado con {len(controller.waypoints)} waypoints")
    return controller


def main():
    """Ejecutar todos los ejemplos."""
    print("🧪 EJEMPLOS DE CREACIÓN DE WAYPOINTS FUERA DEL CONTROLADOR")
    print("=" * 60)
    
    # Ejecutar ejemplos
    example_square_route()
    example_circular_route()
    example_custom_waypoints()
    example_line_route()
    example_rectangular_route()
    example_combined_waypoints()
    
    print("\n✅ Todos los ejemplos ejecutados correctamente")
    print("📋 RESUMEN: Los waypoints se crean fuera del WaypointNavigationController")
    print("    usando la clase Waypoints y se pasan como parámetro al constructor.")


if __name__ == "__main__":
    main()
