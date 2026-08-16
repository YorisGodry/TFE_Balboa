import os, sys
script_dir = os.path.dirname(os.path.abspath(__file__))
src_dir = os.path.abspath(os.path.join(script_dir, "../core"))
sys.path.append(src_dir)
helper_dir = os.path.abspath(os.path.join(script_dir, "../control"))
sys.path.append(helper_dir)

from main import Robot, EmptyBehavior #type:ignore
import constants #type:ignore
from utils import Clock #type:ignore


constants.set_robot_width(104)

with open("sensing_manually.csv", "w") as file:
    file.write("angle,s0,s1,s2,s3,s4\n")
    behavior = EmptyBehavior()
    robot = Robot()
    clock = Clock()
    while True:
        robot.loop(behavior, 1/70)
        text = f"{robot.angle.value}"
        values = ",".join([str(value) for value in robot.line_sensors.sensor_values])
        text += f",{values}"
        file.write(text + "\n")
        clock.add_wait_time(1/70)
        clock.wait()