import math
import numpy as np

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

    def set_cell(self, x, y, value):
        """
        Marca una celda en el mapa usando coordenadas físicas.
        """
        row, col = self.world_to_map(x, y)
        self.grid[row, col] = value

    def get_cell(self, x, y):
        """
        Obtiene el valor de una celda usando coordenadas físicas.
        """
        row, col = self.world_to_map(x, y)
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

