"""
Ejemplo demostrando la flexibilidad de la nueva jerarquía de interfaces.
"""

from robot_simur_uo.interfaces import IRobotBase, IDifferentialRobot, IAckermannRobot
from robot_simur_uo.utils import SimulatedDifferentialRobot, SimulatedAckermannRobot


def demo_robot_polymorphism():
    """Demuestra el polimorfismo con diferentes tipos de robots."""
    
    print("=== Demo de Polimorfismo de Robots ===\n")
    
    # Lista de robots de diferentes tipos
    robots = [
        SimulatedDifferentialRobot(wheel_radius=0.025, wheel_base=0.1),
        SimulatedAckermannRobot(wheelbase=0.25, max_steering_angle=0.52)  # ~30 grados
    ]
    
    # Función que trabaja con cualquier robot (usa IRobotBase)
    def test_basic_movement(robot: IRobotBase, robot_name: str):
        print(f"--- Probando {robot_name} ---")
        print(f"Pose inicial: {robot.get_pose()}")
        
        # Métodos comunes a todos los robots
        robot.move_forward(1.0)
        robot.step(0.5)  # Medio segundo
        print(f"Después de mover adelante: {robot.get_pose()}")
        
        robot.turn_left(1.0)
        robot.step(0.5)
        print(f"Después de girar izquierda: {robot.get_pose()}")
        
        robot.stop()
        print(f"Robot detenido: {robot}")
        print()
    
    # Función específica para robots diferenciales
    def test_differential_robot(robot: IDifferentialRobot):
        print("--- Test específico de robot diferencial ---")
        robot.set_motor_speeds(2.0, 1.0)  # Giro a la derecha
        robot.step(0.5)
        left_speed, right_speed = robot.get_motor_speeds()
        print(f"Velocidades de motores: izq={left_speed:.2f}, der={right_speed:.2f}")
        print(f"Pose después del giro: {robot.get_pose()}")
        print()
    
    # Función específica para robots Ackermann
    def test_ackermann_robot(robot: IAckermannRobot):
        print("--- Test específico de robot Ackermann ---")
        robot.set_steering_angle(0.3)  # ~17 grados
        robot.set_drive_speed(1.5)
        robot.step(0.5)
        print(f"Ángulo dirección: {robot.get_steering_angle():.3f} rad")
        print(f"Velocidad tracción: {robot.get_drive_speed():.2f} m/s")
        print(f"Pose después del movimiento: {robot.get_pose()}")
        print()
    
    # Probar métodos comunes con polimorfismo
    for i, robot in enumerate(robots):
        robot_type = type(robot).__name__
        test_basic_movement(robot, f"Robot {i+1} ({robot_type})")
    
    # Probar métodos específicos usando isinstance
    for robot in robots:
        if isinstance(robot, IDifferentialRobot):
            test_differential_robot(robot)
        elif isinstance(robot, IAckermannRobot):
            test_ackermann_robot(robot)
    
    print("🎉 ¡Demo completado! La jerarquía permite:")
    print("  ✅ Polimorfismo: misma interfaz para robots diferentes")
    print("  ✅ Especialización: métodos específicos por tipo de robot")
    print("  ✅ Extensibilidad: fácil agregar nuevos tipos de robot")


if __name__ == "__main__":
    demo_robot_polymorphism()
