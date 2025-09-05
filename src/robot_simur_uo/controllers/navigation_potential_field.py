import math
from .navigation import NavigationController
from typing import Tuple

class NavigationPotentialFieldController(NavigationController):
    """
    Controlador de navegación usando campos potenciales.

    Ejemplo:
        >>> ctrl = NavigationPotentialFieldController()
        >>> ctrl.set_target(1.0, 2.0)
        >>> v, w = ctrl.calculate_control_commands(0, 0, 0, 0.5, 0.5)
        >>> print(v, w)
    """
    def __init__(self, linear_gain: float = 2.0, steering_gain: float = 2.0,
                 attraction_gain: float = 0.1, repulsion_gain: float = 0.1, repulsion_threshold: float = 2.0):
        super().__init__(linear_gain, steering_gain)
        self.attraction_gain = attraction_gain
        self.repulsion_gain = repulsion_gain
        self.repulsion_threshold = repulsion_threshold

    def calculate_control_commands(self, current_x: float, current_y: float, current_angle: float,
                                  obstacle_x: float, obstacle_y: float) -> Tuple[float, float]:
        """
        Calcula comandos de control considerando el objetivo y el obstáculo más cercano (todo en global).
        Args:
            current_x, current_y, current_angle: Pose actual del robot
            obstacle_x, obstacle_y: Posición global del obstáculo más cercano
        Returns:
            Tuple con (velocidad_lineal, velocidad_de_dirección)
        """
        print(f"[PF] Pose actual: x={current_x:.3f}, y={current_y:.3f}, theta={current_angle:.3f}")
        print(f"[PF] Obstáculo global: x={obstacle_x:.3f}, y={obstacle_y:.3f}")
        # Fuerza atractiva hacia el objetivo
        dx = self.target_x - current_x
        dy = self.target_y - current_y
        attractive_x = self.attraction_gain * dx
        attractive_y = self.attraction_gain * dy
        print(f"\t[PF] Fuerza atractiva: ax={attractive_x:.3f}, ay={attractive_y:.3f}")

        # Fuerza repulsiva por obstáculo
        obs_dx = obstacle_x - current_x
        obs_dy = obstacle_y - current_y
        obs_dist = math.sqrt(obs_dx**2 + obs_dy**2)
        
        repulsive_x, repulsive_y = 0.0, 0.0
        
        if 0 < obs_dist < self.repulsion_threshold:
            factor = (1 / self.repulsion_threshold - 1 / obs_dist) * (1 / obs_dist)**3 * (-2)
            repulsive_x = factor * obs_dx
            repulsive_y = factor * obs_dy
            print(f"[PF] \tFuerza repulsiva: rx={repulsive_x:.3f}, ry={repulsive_y:.3f}, factor={factor:.3f}")
        else:
            print(f"[PF] \tSin fuerza repulsiva (distancia={obs_dist:.3f})")

        # Suma de fuerzas
        total_x = attractive_x - self.repulsion_gain * repulsive_x
        total_y = attractive_y - self.repulsion_gain * repulsive_y
        goal_x = current_x + total_x
        goal_y = current_y + total_y
        print(f"\t[PF] Goal virtual: x={goal_x:.3f}, y={goal_y:.3f}")

        # Dirección y distancia al "goal" virtual
        distance = math.sqrt((goal_x - current_x)**2 + (goal_y - current_y)**2)
        theta_goal = math.atan2(goal_y - current_y, goal_x - current_x)
        angle_diff = math.atan2(math.sin(theta_goal - current_angle), math.cos(theta_goal - current_angle))
        print(f"\t[PF] Distancia a goal: {distance:.3f}, theta_goal={theta_goal:.3f}, angle_diff={angle_diff:.3f}")

        drive_speed = distance * self.linear_gain
        steering_speed = angle_diff * self.steering_gain
        print(f"\t[PF] Comandos finales: drive_speed={drive_speed:.3f}, steering_speed={steering_speed:.3f}")
        return drive_speed, steering_speed

