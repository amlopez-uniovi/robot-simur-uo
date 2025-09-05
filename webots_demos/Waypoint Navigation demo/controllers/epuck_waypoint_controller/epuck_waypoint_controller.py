sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

"""
Script de entrada para ejecutar la demo de navegación por waypoints usando el robot EPuck.

Ejemplo:
    $ python epuck_waypoint_controller.py
"""

from robot_simur_uo.webots.epuck_robot import EPuck
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from waypoint_navigation_demo_controller import run_waypoint_navigation_demo

if __name__ == "__main__":
    run_waypoint_navigation_demo(EPuck)
