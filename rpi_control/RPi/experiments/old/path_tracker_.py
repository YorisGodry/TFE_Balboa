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


position_goals = [(0, 0), (600, 0), (300, -300), (300, -150),  (0, -300)]
theta_goals = [0, 0, pi, pi, pi]
position_goals = [(0, 0), (300, 0), (300, 300), (0, 300)]
theta_goals = [-pi/2, 0, pi/2, pi]
path_repeat = True
behavior = PathTracker(position_goals, theta_goals, path_repeat)
robot = Robot()
clock = Clock()
while True:
    robot.loop(behavior, constants.iteration_duration)
    print(robot.position.x, robot.position.y, robot.angle.value / pi * 180)
    clock.add_wait_time(constants.iteration_duration)
    clock.wait()