"""
Script de entrada para ejecutar la demo de navegación por campo potencial con el robot EPuck.

Ejemplo:
    $ python epuck_navigation_potential_field_controller.py
"""

from robot_simur_uo.webots.epuck_robot import EPuck
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from navigation_potential_field_controller import run_navigation_potential_field

if __name__ == "__main__":
    run_navigation_potential_field(EPuck)