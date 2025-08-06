from robot_simur_uo.webots.rosbot_robot import RosBot
import sys
import os

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from waypoint_navigation_demo_controller import run_waypoint_navigation_demo

if __name__ == "__main__":
    run_waypoint_navigation_demo(RosBot)
