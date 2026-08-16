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


position_goals = [(0, 0), (1, 0)]
theta_goals = [0, 0]
path_repeat = False

with open("temp.csv", "w") as file:
    behavior = PathTracker(position_goals, theta_goals, path_repeat)
    robot = Robot(104)
    clock = Clock()

    i = 0

    while True:
        i  = (i+1) % 70
        if i == 0:
            print("position", robot.dwm.position)
            print("distance", robot.dwm.distance)
        robot.loop(behavior, 1/70)
        clock.add_wait_time(1/70)
        clock.wait()