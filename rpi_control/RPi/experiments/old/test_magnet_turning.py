import importer
from main import Robot #type:ignore
from turning import TurningBalancer #type:ignore
import constants #type:ignore
from utils import Clock #type:ignore

from math import pi


def main(robot, file):
    behavior = TurningBalancer()
    clock = Clock()
    
    columns = "mag_orient,ref_orient,orient".split(",")
    gather = {"active":        lambda robot: robot.active,
              "angle":         lambda robot: robot.angle.value/pi*180,
              "line_position": lambda robot: robot.line_sensors.line_position.value,
              "x":             lambda robot: robot.position.get_relative_xy()[0],
              "y":             lambda robot: robot.position.get_relative_xy()[1],
              "theta":         lambda robot: robot.position.get_theta()/pi*180,
              "raw_mag_x":     lambda robot: robot.lis3mdl.m[0],
              "raw_mag_y":     lambda robot: robot.lis3mdl.m[1],
              "raw_mag_z":     lambda robot: robot.lis3mdl.m[2],
              "calib_mag_orient":  lambda robot: robot.magnetometer.get_orientation(),
              "ref_orient": lambda robot: behavior.ref_orientation,
              "orient": lambda robot: robot.position.get_absolute()[2]}
    for i in range(5):
        gather[f"s{i}"] =      lambda robot: robot.line_sensors.sensor_values[i]
    
    file.write(",".join(columns) + "\n")
    while True:
        robot.loop(behavior, constants.iteration_duration)

        # data gathering
        data = [gather[column](robot) for column in columns]
        text = ",".join([str(entry) for entry in data])
        print(text)
        file.write(text + "\n")

        clock.add_wait_time(constants.iteration_duration)
        clock.wait()

try:
    robot = Robot(104, False, 3)
    with open("temp.csv", "w") as file:
        main(robot, file)
finally:
    robot.motors.reset()