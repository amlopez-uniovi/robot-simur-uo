import math
from numpy import round

import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from coordinates import polar_to_cartesian, transform_points

def main():
    # Medidas polares (ángulo en radianes, distancia en metros)
    medidas = [
        (0.0, 1.0),
        (math.pi/2, 2),
        (math.pi, 4),
        (-math.pi/2, 7)
    ]
    print("Medidas polares:", medidas)

    # 1. Convertir a cartesianas locales
    cartesianas_locales = polar_to_cartesian(medidas)
    print(",\n---\n Coordenadas locales:\n", round(cartesianas_locales))

    ######
    
    # 2. Pose del robot en el mundo (x, y, theta)
    pose_robot = (0.0, 0.0, math.pi/2)
    print("\n---\n Pose del robot:", pose_robot)

    # 3. Transformar a globales
    cartesianas_1 = transform_points(cartesianas_locales, pose_robot)
    print("\n---\n Coordenadas globales:\n", round(cartesianas_1))    
    
    ######3

    # 2. Pose del robot en el mundo (x, y, theta)
    pose_robot = (3.0, 2.0, 0)
    print("\n Pose del robot:", pose_robot)

    # 3. Transformar a globales
    cartesianas_2 = transform_points(cartesianas_1, pose_robot)
    print("\n---\n Coordenadas globales:\n", round(cartesianas_2))

    #####
    # 2. Pose del robot en el mundo (x, y, theta)
    pose_robot = (3.0, 2.0, math.pi/2)
    print("\n---\n Pose del robot:", pose_robot)

    # 3. Transformar a globales
    cartesianas_globales = transform_points(cartesianas_locales, pose_robot)
    print("\n---\n Coordenadas globales:\n", round(cartesianas_globales))
    
    
    print("\n")

if __name__ == "__main__":
    main()
