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

filename = os.path.abspath(os.path.join(script_dir, "../../line_following.csv"))

"""
class FakeAngle(Signal):
    def __init__(self, init_angle, filename):
        super().__init__()
        self.value = init_angle

        self.angles = []
        with open(filename, "r") as file:
            file.readline()
            for line in file.readlines():
                line = line.strip().split(",")
                self.angles.append(float(line[1]) / 180 * pi)
        self.index = 0

    def update(self, accelerometer, gyroscope, delta_time):
        if self.index == len(self.angles):
            return pi/2
        angle = self.angles[self.index]
        self.index += 1
        self.extend(angle, delta_time)

robot.angle = FakeAngle(robot.angle.value, filename)
"""

class FakeLineSensors:
    def __init__(self, filename):
        self.line_position = Signal()
        self.sensor_values = self.read_sensor_values()

        wanted_strip_index = 5
        current_strip_index = -1
        previous_active = False
        self.line_positions = []
        with open(filename, "r") as file:
            file.readline()
            for line in file.readlines():
                line = line.strip().split(",")
                active = line[0] == "True"
                if not active and not previous_active:
                    continue
                elif active and previous_active and current_strip_index == wanted_strip_index:
                    self.line_positions.append(float(line[-1]))
                elif active and previous_active and current_strip_index != wanted_strip_index:
                    continue
                elif active and not previous_active:
                    current_strip_index += 1
                    if current_strip_index == wanted_strip_index:
                        self.line_positions.append(float(line[-1]))
                    previous_active = active
                elif not active and previous_active:
                    if wanted_strip_index == current_strip_index:
                        break
                    previous_active = active
                    continue
        self.index = 0

    def read_sensor_values(self):
        return [0, 0, 0, 0, 0]
    
    def update(self, delta_time):
        if not robot.active:
            self.line_position.extend(0, delta_time)
            return
        if self.index == len(self.line_positions):
            raise Exception("End of file")
        error = self.line_positions[self.index]
        self.index += 1
        self.line_position.extend(error, delta_time)

    def reset(self):
        self.line_position.reset()
        self.index = 0

robot.line_sensors = FakeLineSensors(filename)

clock = Clock()
with open("line_follower_recreation.csv", "w") as file:
    file.write("active,angle,s0,s1,s2,s3,s4,line_position,x,y,theta\n")
    while True:
        robot.loop(behavior, 1/70)
        text = f"{robot.active},{round(robot.angle.value/pi*180, 3)},"
        s = robot.line_sensors.sensor_values
        for i in range(5):
            text += f"{s[i]},"
        text += f"{robot.line_sensors.line_position.value},"
        text += f"{robot.position.x},{robot.position.y},{robot.position.theta/pi*180}"
        file.write(text + "\n")
        clock.add_wait_time(1/70)
        clock.wait()

