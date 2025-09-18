"""
Script de entrada para ejecutar la demo básica con el robot EPuck.

Ejemplo:
    $ python epuck_basic_controller.py
"""

from robot_simur_uo.webots.epuck_robot import EPuck
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from basic_navigation_controller import run_basic_demo

if __name__ == "__main__":
    run_basic_demo(EPuck)