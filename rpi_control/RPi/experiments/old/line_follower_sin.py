import os, sys
script_dir = os.path.dirname(os.path.abspath(__file__))
src_dir = os.path.abspath(os.path.join(script_dir, "../core"))
sys.path.append(src_dir)
helper_dir = os.path.abspath(os.path.join(script_dir, "../control"))
sys.path.append(helper_dir)

from signal_ import Signal #type:ignore
from line_follower import LineFollower #type:ignore
from main import Robot #type:ignore
from utils import Clock #type:ignore

from math import pi, sin

behavior = LineFollower()
robot = Robot()

class FakeLineSensors:
    def __init__(self):
        self.line_position = Signal()
        self.sensor_values = self.read_sensor_values()

    def read_sensor_values(self):
        return [0, 0, 0, 0, 0]
    
    def update(self, delta_time):
        x = robot.position.x
        sin_y = sin(x / 1200 * 2 * pi - pi / 2) * 150 + 150
        y = robot.position.y
        error = sin_y - y
        error = error / 50 * -2
        if abs(error) > 2:
            error = 0
        self.line_position.extend(error, delta_time)

    def reset(self):
        self.line_position.reset()

robot.line_sensors = FakeLineSensors()
clock = Clock()
with open("temp.csv", "w") as file:
    file.write("active,angle,s0,s1,s2,s3,s4,line_position\n")
    while True:
        robot.loop(behavior, 1/70)
        text = f"{robot.active},{round(robot.angle.value/pi*180, 3)},"
        s = robot.line_sensors.sensor_values
        for i in range(5):
            text += f"{s[i]},"
        text += f"{robot.line_sensors.line_position.value}"
        print(text)
        file.write(text + "\n")
        clock.add_wait_time(1/70)
        clock.wait()

