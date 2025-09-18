"""
Script de entrada para ejecutar la demo de navegación aleatoria con mapeo usando el robot RosBot.

Ejemplo:
    $ python rosbot_random_navigation_mapping_controller.py
"""

from robot_simur_uo.webots.rosbot_robot import RosBot
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from random_navigation_mapping_demo_controller import run_random_navigation_mapping_demo

if __name__ == "__main__":
    run_random_navigation_mapping_demo(RosBot)
