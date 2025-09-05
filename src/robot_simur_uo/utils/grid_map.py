
import math
import numpy as np

from robot_simur_uo.utils.coordinates import transform_points

def bresenham(start, end):
    """
    Algoritmo de Bresenham para trazar una línea entre dos puntos.
    """
    points = []
    x1, y1 = start
    x2, y2 = end
    dx = abs(x2 - x1)
    dy = abs(y2 - y1)
    sx = 1 if x1 < x2 else -1
    sy = 1 if y1 < y2 else -1
    if dx > dy:
        err = dx / 2
        while x1 != x2:
            points.append((x1, y1))
            err -= dy
            if err < 0:
                y1 += sy
                err += dx
            x1 += sx
    else:
        err = dy / 2
        while y1 != y2:
            points.append((x1, y1))
            err -= dx
            if err < 0:
                x1 += sx
                err += dy
            y1 += sy
    points.append((x2, y2))
    return points

class GridMap:

        
    def __init__(self, bottom_left, top_right, resolution, empty_value=0.0):
        """
        Inicializa el mapa de rejilla.
        Args:
            bottom_left (tuple): (x, y) esquina inferior izquierda (metros)
            top_right (tuple): (x, y) esquina superior derecha (metros)
            resolution (float): tamaño de celda en metros
        """
        self.bottom_left = bottom_left
        self.top_right = top_right
        self.resolution = resolution
        # Calcular dimensiones del mapa (redondeo superior)
        width = top_right[0] - bottom_left[0]
        height = top_right[1] - bottom_left[1]
        self.cols = math.ceil(width / resolution)
        self.rows = math.ceil(height / resolution)
        self.grid = np.zeros((self.rows, self.cols), dtype=np.int8)
        self.empty_value = empty_value
        self.grid[:, :] = self.empty_value


    def reset(self):
        self.grid[:, :] = self.empty_value

    def world_to_map(self, x, y):
        """
        Convierte coordenadas físicas (x, y) a índices de celda (fila, columna).
        La columna crece hacia la derecha, la fila crece hacia arriba (fila 0 = parte inferior).
        """
        col = int(np.floor((x - self.bottom_left[0]) / self.resolution))
        # Fila invertida: la fila 0 es la parte inferior (mínimo y)
        row = self.rows - 1 - int(np.floor((y - self.bottom_left[1]) / self.resolution))
        col = min(max(col, 0), self.cols - 1)
        row = min(max(row, 0), self.rows - 1)
        return row, col

    def map_to_world(self, row, col):
        """
        Convierte índices de celda (fila, columna) a coordenadas físicas (x, y) del centro de la celda.
        """
        x = self.bottom_left[0] + (col + 0.5) * self.resolution
        y = self.bottom_left[1] + ((self.rows - 1 - row) + 0.5) * self.resolution
        return x, y

    def set_cell(self, x, y, value, world_coordinates=True):
        """
        Marca una celda en el mapa usando coordenadas físicas.
        """
        if world_coordinates:
            row, col = self.world_to_map(x, y)
        else:
            row, col = x, y
            col = min(max(col, 0), self.cols - 1)
            row = min(max(row, 0), self.rows - 1)
        
        self.grid[row, col] = value

    def get_cell(self, x, y, world_coordinates=True):
        """
        Obtiene el valor de una celda usando coordenadas físicas.
        """
        if world_coordinates:
            row, col = self.world_to_map(x, y)
        else:
            row, col = x, y
            col = min(max(col, 0), self.cols - 1)
            row = min(max(row, 0), self.rows - 1)
            
        return self.grid[row, col]

    def __repr__(self):
        return f"GridMap({self.rows}x{self.cols}, res={self.resolution}, bottom_left={self.bottom_left}, top_right={self.top_right})"

    def visualize(self, cmap='gray', fig=None, block=False):
        """
        Visualiza la matriz como un mapa de grises, con correspondencia física correcta.
        Requiere matplotlib.
        Si se proporciona una figura, dibuja sobre ella. Si no, la crea.
        Devuelve la figura.
        El parámetro 'block' controla si plt.show() es bloqueante.
        """
        import matplotlib.pyplot as plt
        if fig is None:
            fig = plt.figure()
        ax = fig.gca()
        im = ax.imshow(self.grid, cmap=cmap, origin='upper',
                       extent=[self.bottom_left[0], self.top_right[0], self.bottom_left[1], self.top_right[1]])
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_title('GridMap (mapa de grises)')
        #fig.colorbar(im, ax=ax, label='Valor celda')
        plt.show(block=block)
        plt.pause(0.001)  # Pequeña pausa para actualizar la figura
        return fig
    
    def get_occupied_free_cells_from_pose_obstacles(self, pose, obstacle_points, free_points, points_in_robot_frame=False):
        """
        Devuelve dos listas:
        - occupied: celdas ocupadas por obstáculos
        - free: celdas libres recorridas por los rayos del LiDAR
        Si los puntos están en el marco del robot, se transforman al marco global usando la pose.
        """
        # Transformar puntos si es necesario
        if points_in_robot_frame:
            obstacle_points = transform_points(obstacle_points, pose)
            free_points = transform_points(free_points, pose)

        # Obtener celda del robot
        pose_row, pose_col = self.world_to_map(pose[0], pose[1])

        occupied_set = set()
        free_set = set()

        # Marcar celdas ocupadas y los rayos hasta ellas
        for point in obstacle_points:
            cell = self.world_to_map(point[0], point[1])
            occupied_set.add(cell)
            for c in bresenham((pose_row, pose_col), cell):
                free_set.add(c)

        # Marcar celdas libres recorridas por rayos sin obstáculo
        for point in free_points:
            cell = self.world_to_map(point[0], point[1])
            for c in bresenham((pose_row, pose_col), cell):
                free_set.add(c)

        # Las celdas ocupadas no son libres
        free = list(free_set - occupied_set)
        occupied = list(occupied_set)
        
        return occupied, free

if __name__ == "__main__":
    # Ejemplo de uso
    bottom_left = (-2.0, -3.0)
    top_right = (2.0, 4.0)
    resolution = 1.0
    grid = GridMap(bottom_left, top_right, resolution)
    print(grid)
    # Recorrer todas las celdas y visualizar cada centro
    import time
    import matplotlib.pyplot as plt

    # Recorrer todas las celdas y visualizar cada centro
    for row in range(grid.rows):
        for col in range(grid.cols):
            grid.grid[:, :] = 0
            x, y = grid.map_to_world(row, col)
            grid.grid[row, col] = 1
            plt.imshow(grid.grid, cmap='gray', origin='upper',
                       extent=[grid.bottom_left[0], grid.top_right[0], grid.bottom_left[1], grid.top_right[1]])
            plt.xlabel('X (m)')
            plt.ylabel('Y (m)')
            plt.title(f'Celda activa: centro=({x:.2f}, {y:.2f})')
            plt.colorbar(label='Valor celda')
            plt.pause(0.2)
            plt.clf()

    # Recorrer coordenadas del mundo con paso 'step'
    step = 0.5  # Puedes ajustar el paso aquí
    x_min, x_max = grid.bottom_left[0], grid.top_right[0]
    y_min, y_max = grid.bottom_left[1], grid.top_right[1]
    x_vals = np.arange(x_min, x_max, step)
    y_vals = np.arange(y_min, y_max, step)
    for x in x_vals:
        for y in y_vals:
            grid.grid[:, :] = 0
            row, col = grid.world_to_map(x, y)
            grid.grid[row, col] = 1
            plt.imshow(grid.grid, cmap='gray', origin='upper',
                       extent=[grid.bottom_left[0], grid.top_right[0], grid.bottom_left[1], grid.top_right[1]])
            plt.xlabel('X (m)')
            plt.ylabel('Y (m)')
            plt.title(f'Coordenada activa: ({x:.2f}, {y:.2f})')
            plt.colorbar(label='Valor celda')
            plt.pause(0.2)
            plt.clf()

    plt.close()

