import importer
from main import Robot, EmptyBehavior #type:ignore
import constants #type:ignore
from utils import Clock #type:ignore

from math import pi

iter_time = 1/10

def main(robot, file):
    behavior = EmptyBehavior()
    clock = Clock()
    
    columns = "raw_mag_x,raw_mag_y,raw_mag_z,calib_orient".split(",")
    gather = {"active":        lambda robot: robot.active,
              "angle":         lambda robot: robot.angle.value/pi*180,
              "line_position": lambda robot: robot.line_sensors.line_position.value,
              "x":             lambda robot: robot.position.get_relative_xy()[0],
              "y":             lambda robot: robot.position.get_relative_xy()[1],
              "theta":         lambda robot: robot.position.get_theta()/pi*180,
              "raw_mag_x":     lambda robot: robot.lis3mdl.m[0],
              "raw_mag_y":     lambda robot: robot.lis3mdl.m[1],
              "raw_mag_z":     lambda robot: robot.lis3mdl.m[2],
              "calib_orient":  lambda robot: robot.magnetometer.get_orientation()}
    for i in range(5):
        gather[f"s{i}"] =      lambda robot: robot.line_sensors.sensor_values[i]
    
    file.write(",".join(columns) + "\n")
    while True:
        robot.loop(behavior, iter_time)
        robot.lis3mdl.read()

        # data gathering
        data = [gather[column](robot) for column in columns]
        text = ",".join([str(entry) for entry in data])
        print(text)
        file.write(text + "\n")

        clock.add_wait_time(iter_time)
        clock.wait()

try:
    robot = Robot(104, False, 0)
    with open("temp.csv", "w") as file:
        main(robot, file)
finally:
    robot.motors.reset()