
import numpy as np
from .grid_map import GridMap

class ProbabilisticGridMap(GridMap):

    def __init__(self, bottom_left, top_right, resolution, empty_value=0.5, occupancy_factor=0.7, free_factor=0.3):
        super().__init__(bottom_left, top_right, resolution, empty_value)
        # Inicializa la grid como probabilidades (float entre 0 y 1)
        self.grid = np.full((self.rows, self.cols), empty_value, dtype=np.float32)  # 0.5 = desconocido
        self.occupancy_factor = occupancy_factor  # Factor de fusión para actualización probabilística
        self.free_factor = free_factor  # Factor de fusión para actualización probabilística

    # Aquí puedes añadir métodos específicos para actualización probabilística, etc.

    def update(self, obstacle_points, free_points, pose, points_in_robot_frame=True):
        """
        Actualiza el mapa probabilístico usando los puntos de obstáculos y libres.

        Args:
            obstacle_points (list): Lista de (x, y) de obstáculos.
            free_points (list): Lista de (x, y) de puntos libres.
            pose (tuple): Pose (x, y, theta) del robot en el marco global.
            points_in_robot_frame (bool): Si True, transforma los puntos al marco global.
        """

        celdas_ocupadas, celdas_libres = self.get_occupied_free_cells_from_pose_obstacles(
            pose, obstacle_points, free_points, points_in_robot_frame=points_in_robot_frame
        )

        # Actualizar celdas libres
        for cell in celdas_libres:
            self._update_cell_probability(cell, self.free_factor)

        # Actualizar celdas ocupadas
        for cell in celdas_ocupadas:
            self._update_cell_probability(cell, self.occupancy_factor)

    def _update_cell_probability(self, cell, p, alpha=0.6):
        """
        Actualiza la probabilidad de una celda usando un promedio ponderado (fusión simple).

        Args:
            cell (tuple): Índice de la celda (row, col).
            p (float): Probabilidad a fusionar.
            alpha (float): Peso de la actualización.
        """
        row, col = cell
        self.grid[row, col] = (1 - alpha) * self.grid[row, col] + alpha * p
