
"""
Script de entrada para ejecutar la demo de test de sensores usando el robot EPuck.

Ejemplo:
    $ python epuck_sensor_test_controller.py
"""

from robot_simur_uo.webots.epuck_robot import EPuck
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from sensor_test_demo_controller import run_sensor_test_demo

if __name__ == "__main__":
    run_sensor_test_demo(EPuck)
