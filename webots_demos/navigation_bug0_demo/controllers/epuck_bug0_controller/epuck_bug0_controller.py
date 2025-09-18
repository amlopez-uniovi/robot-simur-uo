"""
Script de entrada para ejecutar la demo Bug0 con el robot EPuck.

Ejemplo:
    $ python epuck_bug0_controller.py
"""

from robot_simur_uo.webots.epuck_robot import EPuck
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from bug0_demo_controller import run_bug0_demo

if __name__ == "__main__":
    run_bug0_demo(EPuck)