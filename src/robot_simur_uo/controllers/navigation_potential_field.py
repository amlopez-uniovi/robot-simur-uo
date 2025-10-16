import math
from .navigation_lookahead import NavigationLookAhead
from typing import Tuple

class NavigationPotentialFieldController(NavigationLookAhead):
    """
    Controlador de navegación usando campos potenciales.

    Ejemplo:
        >>> ctrl = NavigationPotentialFieldController()
        >>> ctrl.set_target(1.0, 2.0)
        >>> v, w = ctrl.calculate_control_commands(0, 0, 0, 0.5, 0.5)
        >>> print(v, w)
    """
    def __init__(self, linear_gain: float = 2.0, steering_gain: float = 2.0,
                 attraction_gain: float = 0.1, repulsion_gain: float = 0.1, repulsion_threshold: float = 2.0, look_ahead_distance: float = 0.5):
        super().__init__(linear_gain, steering_gain, look_ahead_distance)
        self.attraction_gain = attraction_gain
        self.repulsion_gain = repulsion_gain
        self.repulsion_threshold = repulsion_threshold

    def calculate_goal_virtual(self, current_x: float, current_y: float, obstacle_x: float, obstacle_y: float) -> Tuple[float, float]:
        # Fuerza atractiva hacia el objetivo
        dx = current_x - self.target_x 
        dy = current_y - self.target_y 
        
        attractive_x = self.attraction_gain * dx
        attractive_y = self.attraction_gain * dy
        print(f"\t[PF] Fuerza atractiva: ax={attractive_x:.3f}, ay={attractive_y:.3f}")

        # Fuerza repulsiva por obstáculo
        obs_dx = obstacle_x - current_x
        obs_dy = obstacle_y - current_y
        obs_dist = math.sqrt(obs_dx**2 + obs_dy**2)
        
        repulsive_x, repulsive_y = 0.0, 0.0
        
        if 0 < obs_dist < self.repulsion_threshold:
            factor = (1 / self.repulsion_threshold - 1 / obs_dist) * (1 / obs_dist)**2 * (1 / 2 * obs_dist) * (-2)
            repulsive_x = factor * obs_dx
            repulsive_y = factor * obs_dy
            print(f"[PF] \tFuerza repulsiva: rx={repulsive_x:.3f}, ry={repulsive_y:.3f}, factor={factor:.3f}")
        else:
            print(f"[PF] \tSin fuerza repulsiva (distancia={obs_dist:.3f})")

        # Suma de fuerzas
        total_x = -self.attraction_gain * dx - self.repulsion_gain * repulsive_x
        total_y = -self.attraction_gain * dy - self.repulsion_gain * repulsive_y
        
        # Verificar si las fuerzas se cancelan (mínimo local)
        force_magnitude = math.sqrt(total_x**2 + total_y**2)
        if force_magnitude < 0.1:  # Fuerzas casi se cancelan
            print(f"[PF] ⚠️ ADVERTENCIA: Mínimo local detectado - fuerzas se cancelan (magnitud: {force_magnitude:.4f})")
            return current_x, current_y  # Mantener posición actual

        goal_x = current_x + total_x
        goal_y = current_y + total_y
        print(f"\t[PF] Goal virtual: x={goal_x:.3f}, y={goal_y:.3f}")

        return goal_x, goal_y


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
       
        # Verificar si hemos alcanzado el objetivo final
        final_distance = math.sqrt(
            (self.target_x - current_x)**2 + (self.target_y - current_y)**2
        )
        
        if final_distance < self.tolerance:
            return 0.0, 0.0

        # Calcular goal virtual con campos potenciales
        goal_x, goal_y = self.calculate_goal_virtual(current_x, current_y, obstacle_x, obstacle_y)
        
        if goal_x == current_x and goal_y == current_y:
            # Mínimo local detectado, no mover
            return 1.0, 1.0

        # Temporalmente cambiar el target para usar lookahead hacia el goal virtual
        original_target_x, original_target_y = self.target_x, self.target_y
        self.target_x, self.target_y = goal_x, goal_y
        
        drive_speed, steering_speed = super().calculate_control_commands(current_x, current_y, current_angle)
        
         # Restaurar el target original
        self.target_x, self.target_y = original_target_x, original_target_y
                
        print(f"\t[PF] Goal virtual: x={goal_x:.3f}, y={goal_y:.3f}")
       
        return drive_speed, steering_speed


