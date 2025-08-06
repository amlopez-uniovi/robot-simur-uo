"""
Controlador de navegación demo para RosBot
Estructura unificada: inicialización, bucle principal, sensores, control y logging
"""
from robot_simur_uo.webots.rosbot_robot import RosBot
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from navigation_demo_controller import run_navigation_demo

if __name__ == "__main__":
    run_navigation_demo(RosBot)