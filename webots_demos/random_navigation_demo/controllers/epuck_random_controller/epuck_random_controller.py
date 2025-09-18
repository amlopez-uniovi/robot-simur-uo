"""
Script de entrada para ejecutar la demo de navegación aleatoria con el robot EPuck.

Ejemplo:
    $ python epuck_random_controller.py
"""

from robot_simur_uo.webots.epuck_robot import EPuck
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from random_navigation_demo_controller import run_random_navigation_demo

if __name__ == "__main__":
    run_random_navigation_demo(EPuck)
