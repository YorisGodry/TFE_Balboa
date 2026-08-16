import os, sys
script_dir = os.path.dirname(os.path.abspath(__file__))
src_dir = os.path.abspath(os.path.join(script_dir, "../core"))
sys.path.append(src_dir)
helper_dir = os.path.abspath(os.path.join(script_dir, "../control"))
sys.path.append(helper_dir)

import constants #type:ignore
from main import Robot, EmptyBehavior #type:ignore
from utils import Clock #type:ignore

from math import pi


behavior = EmptyBehavior()
robot = Robot()
clock = Clock()
with open("line_following_inactive.csv", "w") as file:
    file.write("active,angle,s0,s1,s2,s3,s4,line_position\n")
    while True:
        robot.loop(behavior, constants.iteration_duration)
        text = f"{robot.active},{round(robot.angle.value/pi*180, 3)},"
        s = robot.line_sensors.sensor_values
        for i in range(5):
            text += f"{s[i]},"
        text += f"{robot.line_sensors.line_position.value}"
        print(text)
        file.write(text + "\n")
        clock.add_wait_time(constants.iteration_duration)
        clock.wait()