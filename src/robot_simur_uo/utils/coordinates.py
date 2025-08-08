def is_angle_in_range(angle, angle_min, angle_max):
    """
    Devuelve True si angle está dentro del rango [angle_min, angle_max] en el círculo trigonométrico.
    Soporta rangos que cruzan el cero y normaliza todo a [0, 2π).
    """
    # Normaliza todos los ángulos al rango [0, 2π)
    angle = angle % (2 * math.pi)
    angle_min = angle_min % (2 * math.pi)
    angle_max = angle_max % (2 * math.pi)

    #print(f"angulo {math.degrees(angle):.1f}°, angulo minimo {math.degrees(angle_min):.1f}°, angulo maximo {math.degrees(angle_max):.1f}°")

    # Log de entrada
    #print(f"[is_angle_in_range] angle={math.degrees(angle):.1f}°, min={math.degrees(angle_min):.1f}°, max={math.degrees(angle_max):.1f}°")

    # Si el rango cubre todo el círculo
    if angle_min == angle_max:
        #print("[is_angle_in_range] Rango cubre todo el círculo (min == max), devuelve True")
        return True

    # Rango normal (no cruza el cero)
    if angle_min < angle_max:
        result = angle_min <= angle <= angle_max
        #print(f"[is_angle_in_range] Rango directo: {result}")
        return result
    else:
        # Rango circular (cruza el cero)
        result = angle >= angle_min or angle <= angle_max
        print(f"[is_angle_in_range] Rango circular: {result}")
        return result


def transform_points(points, pose):
    """
    Transforma una lista de puntos (x, y) locales a coordenadas globales usando la pose del robot.
    Args:
        points (list of tuple): Lista de tuplas (x, y) en el marco local del robot
        pose (tuple): Pose del robot (x, y, theta) en el mundo
    Returns:
        list of tuple: Lista de tuplas (x, y) en el marco global
    """
    x_r, y_r, theta = pose
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)
    transformed = []
    for x, y in points:
        x_w = x_r + x * cos_t - y * sin_t
        y_w = y_r + x * sin_t + y * cos_t
        transformed.append((x_w, y_w))
    return transformed

def polar_to_cartesian(points):
    """
    Convierte una lista de puntos en coordenadas polares (ángulo, distancia)
    a coordenadas cartesianas (x, y).
    Args:
        points (list of tuple): Lista de tuplas (ángulo_rad, distancia_m)
    Returns:
        list of tuple: Lista de tuplas (x, y)
    """
    cartesian = []
    for angle, distance in points:
        # Validar ángulo y distancia
        if not isinstance(angle, (int, float)) or not isinstance(distance, (int, float)):
            continue
        if math.isnan(angle) or math.isnan(distance):
            continue
        if math.isinf(angle) or math.isinf(distance):
            continue
        if distance < 0:
            continue
        x = distance * math.cos(angle)
        y = distance * math.sin(angle)
        cartesian.append((x, y))
    return cartesian
"""
Sistema de coordenadas básico para robots.
"""
import math


class RobotPose:
    """Representa la posición y orientación de un robot en el espacio 2D."""
    
    def __init__(self, x=0.0, y=0.0, theta=0.0):
        """
        Args:
            x (float): Posición en el eje X
            y (float): Posición en el eje Y
            theta (float): Orientación en radianes
        """
        self.x = x
        self.y = y
        self.theta = theta
    
    def distance_to(self, other):
        """Calcula la distancia euclidiana a otra posición."""
        dx = other.x - self.x
        dy = other.y - self.y
        return math.sqrt(dx**2 + dy**2)
    
    def angle_to(self, other):
        """Calcula el ángulo hacia otra posición."""
        dx = other.x - self.x
        dy = other.y - self.y
        return math.atan2(dy, dx)
    
    def copy(self):
        """Crea una copia de la pose."""
        return RobotPose(self.x, self.y, self.theta)
    
    def to_tuple(self):
        """Convierte la pose a una tupla (x, y, theta)."""
        return (self.x, self.y, self.theta)
    
    def update(self, dx, dy, dtheta):
        """Actualiza la pose con cambios incrementales."""
        self.x += dx
        self.y += dy
        self.theta += dtheta
    
    def __str__(self):
        return f"RobotPose(x={self.x:.2f}, y={self.y:.2f}, theta={self.theta:.2f})"
    
    def __repr__(self):
        return self.__str__()
    
    def get_pose(self):
        """Devuelve la pose como una tupla (x, y, theta)."""
        return (self.x, self.y, self.theta)