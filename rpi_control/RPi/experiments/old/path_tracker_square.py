import os, sys
script_dir = os.path.dirname(os.path.abspath(__file__))
src_dir = os.path.abspath(os.path.join(script_dir, "../core"))
sys.path.append(src_dir)
helper_dir = os.path.abspath(os.path.join(script_dir, "../control"))
sys.path.append(helper_dir)

from main import Robot #type:ignore
import constants #type:ignore
from utils import Clock #type:ignore
from path_tracker import PathTracker #type:ignore

from math import pi


position_goals = [(0, 0), (1500, 0), (1500, -600), (0, -600), (0, -900), (-300, -900), (-300, -300), (900, -300)]
theta_goals = [0, 0, -pi/2, -pi, -pi/2, -pi, pi/2, 0]
path_repeat = False

constants.set_robot_width(104)

with open("path_tracker_square.csv", "w") as file:
    file.write("active,state,x,y,theta\n")
    behavior = PathTracker(position_goals, theta_goals, path_repeat)
    robot = Robot()
    clock = Clock()
    while True:
        robot.loop(behavior, 1/70)
        text = f"{1 if robot.active else 0},{behavior.planner.state}"
        text += f",{robot.position.x},{robot.position.y},{robot.position.theta}"
        file.write(text + "\n")
        clock.add_wait_time(1/70)
        clock.wait()