import os, sys
script_dir = os.path.dirname(os.path.abspath(__file__))
src_dir = os.path.abspath(os.path.join(script_dir, "../core"))
sys.path.append(src_dir)
helper_dir = os.path.abspath(os.path.join(script_dir, "../control"))
sys.path.append(helper_dir)


from time import time
from math import pi
import constants #type:ignore

columns = []
header_written = True

gather = {  "active":        lambda robot: robot.active,
            "alpha":         lambda robot: robot.angle.value/pi*180,
            "line_position": lambda robot: robot.line_sensors.line_position.value,
            "x":             lambda robot: robot.position.get_relative()[0],
            "y":             lambda robot: robot.position.get_relative()[1],
            "theta":         lambda robot: robot.position.get_relative()[2]/pi*180,
            "raw_mag_x":     lambda robot: robot.lis3mdl.m[0],
            "raw_mag_y":     lambda robot: robot.lis3mdl.m[1],
            "raw_mag_z":     lambda robot: robot.lis3mdl.m[2],
            "raw_acc_x":     lambda robot: robot.lsm6.a.x,
            "raw_acc_y":     lambda robot: robot.lsm6.a.y,
            "raw_acc_z":     lambda robot: robot.lsm6.a.z,
            "mag_orient":    lambda robot: robot.magnetometer.get_orientation(),
            #"ref_orient": lambda robot: behavior.ref_orientation,
            "orient": lambda robot: robot.position.get_absolute()[2],
            "phase": lambda robot: robot.phase}
gather["left_command"] = lambda robot: robot.motors.previous_left_speed
gather["right_command"] = lambda robot: robot.motors.previous_right_speed
gather["left_speed"] = lambda robot: robot.encoders.left.derivative
gather["right_speed"] = lambda robot: robot.encoders.right.derivative
for i in range(5):
    gather[f"s{i}"] =      lambda robot: robot.line_sensors.sensor_values[i]

def set_columns(columns_):
    global columns, header_written
    header_written = False
    columns = columns_.split(",")

show_columns = True
def write_columns(file, robot):
    if len(columns) == 0:
        return

    global header_written
    if not header_written:
        file.write(",".join(columns) + "\n")
        header_written = True
    
    data = [gather[column](robot) for column in columns]
    text = ",".join([str(entry) for entry in data])
    if show_columns:
        print(text)
    file.write(text + "\n")

show_compute_time = False
def print_compute_time(clock):
    if not show_compute_time:
        return
    used = time() - clock.wait_until_time
    total = constants.iteration_duration
    percent_used = round(used / total * 100, 1)
    used = round(used * 1000, 3)
    total = round(total * 1000, 3)
    print(f"{used} ms / {total} ms ({percent_used}%)")


















from motors import LinRegMotors #type:ignore
def set_lin_reg_motors(robot):
    robot.motors = LinRegMotors(robot.balboa)

from main import DWMPosition #type:ignore
class DualPosition(DWMPosition):
    def __init__(self, robot, dwm_active, mag_active):
        super().__init__(robot, dwm_active, mag_active)
        self.correct_position = DWMPosition(robot, dwm_active, mag_active)
    def reset(self):
        super().reset()
        self.correct_position.reset()
    def update_no_initialization(self, robot, delta_time):
        super().update_no_initialization(robot, delta_time)
        self.correct_position.update_no_initialization(robot, delta_time)
    def is_mag_ready(self):
        return super().is_mag_ready() and self.correct_position.is_mag_ready()
    def is_dwm_ready(self):
        return super().is_dwm_ready() and self.correct_position.is_dwm_ready()
    def update(self, robot, delta_time):
        super().update(robot, delta_time)
        self.correct_position.update(robot, delta_time)
    def is_initialized(self):
        return super().is_initialized() and self.correct_position.is_initialized()
def set_dual_position(robot, dwm_active, mag_active):
    robot.position = DualPosition(robot, dwm_active, mag_active)

def create_planner(name):
    raise NotImplementedError

from main import Robot #type:ignore
def create_robot(width, dwm_active, mag_active):
    return Robot(width, dwm_active, mag_active)

from utils import Clock #type:ignore
def main(robot, behavior, on_loop=None):
    try:
        clock = Clock()
        with open("temp.csv", "w") as file:
            print("READY - Robot is ready to be raised")
            while True:
                robot.loop(behavior, constants.iteration_duration)
                write_columns(file, robot)
                print_compute_time(clock)
                clock.wait(constants.iteration_duration)
    except KeyboardInterrupt:
        pass
    finally:
        robot.motors.reset()


""" Template of experiments:

from path_tracker import ReferencePointBehavior #type:ignore
import constants #type:ignore
from helper import *

robot = create_robot(104, False, False)
behavior = ReferencePointBehavior(create_planner("NAME"))
set_columns("COLUMNS")
main(robot, behavior)

"""