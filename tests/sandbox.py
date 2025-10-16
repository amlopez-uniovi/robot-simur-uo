import numpy as np

vector = np.array([1, 2, 3, 0, 0, 5, 5, 5, 5, 0, 1, 1])

vector2 = np.clip(vector, 0, 1)

v3 = np.concatenate(([0], vector2, [0]))

difer = np.diff(v3)

inicio_gaps = np.where(difer > 0)[0]
fin_gaps = np.where(difer == -1)[0]

print(f"Inicio = {inicio_gaps}, fin = {fin_gaps}")

longitudes = fin_gaps - inicio_gaps

print(f"Longitudes = {longitudes}")

max_long = np.max(longitudes)
max_idx = np.argmax(longitudes)
print(f"Max longitud = {max_long}, índice = {max_idx}")

indice_inicio_max_gap = inicio_gaps[max_idx]
indice_fin_max_gap = fin_gaps[max_idx]-1
print(f"Inicio del gap más largo = {indice_inicio_max_gap}, fin = {indice_fin_max_gap}")
