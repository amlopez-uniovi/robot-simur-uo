"""
Controlador de navegación campo potencial para EPuck
Estructura unificada: inicialización, bucle principal, sensores, control y logging
"""
from robot_simur_uo.webots.epuck_robot import EPuck
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from navigation_potential_field_controller import run_navigation_potential_field

if __name__ == "__main__":
    run_navigation_potential_field(EPuck)