import numpy as np
from .probabilistic_grid_map import ProbabilisticGridMap

class ProbabilisticLogOddsGridMap(ProbabilisticGridMap):
    def __init__(self, bottom_left, top_right, resolution, empty_value=0.0, occupancy_factor=1.95, free_factor=-1.1):
        super().__init__(bottom_left, top_right, resolution, empty_value, occupancy_factor, free_factor)
        # Internamente, la grid almacena log-odds
        self.grid = np.full((self.rows, self.cols), self.prob_to_logodds(empty_value), dtype=np.float32)

    @staticmethod
    def prob_to_logodds(p):
        """
        Convierte probabilidad a log-odds.

        Args:
            p (float): Probabilidad (0-1).

        Returns:
            float: Log-odds correspondiente.
        """
        p = np.clip(p, 1e-6, 1-1e-6)
        return np.log(p / (1 - p))

    @staticmethod
    def logodds_to_prob(l):
        """
        Convierte log-odds a probabilidad.

        Args:
            l (float): Log-odds.

        Returns:
            float: Probabilidad correspondiente (0-1).
        """
        return 1 / (1 + np.exp(-l))

    def _update_cell_probability(self, cell, p, alpha=1.0):
        """
        Actualiza la celda usando suma de log-odds (Bayesiano).

        Args:
            cell (tuple): Índice de la celda (row, col).
            p (float): Log-odds a sumar.
            alpha (float): Ignorado (compatibilidad).
        """
        row, col = cell
        self.grid[row, col] += p

    def get_probability_grid(self):
        """
        Devuelve la grid en probabilidades (para visualización o uso externo).

        Returns:
            np.ndarray: Grid de probabilidades (valores entre 0 y 1).
        """
        return self.logodds_to_prob(self.grid)
