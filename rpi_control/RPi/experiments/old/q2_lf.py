import importer
from main import Robot #type:ignore
import constants #type:ignore
from utils import Clock #type:ignore
from line_follower import LineFollower #type:ignore

from math import pi


def main(robot, file):
    behavior = LineFollower()
    clock = Clock()
    
    columns = "active,angle,s0,s1,s2,s3,s4,line_position,x,y,theta".split(",")
    gather = {"active": lambda robot: robot.active,
              "angle": lambda robot: robot.angle.value/pi*180,
              "line_position": lambda robot: robot.line_sensors.line_position.value,
              "x": lambda robot: robot.position.get_relative_xy()[0],
              "y": lambda robot: robot.position.get_relative_xy()[1],
              "theta": lambda robot: robot.position.get_theta()/pi*180}
    for i in range(5):
        gather[f"s{i}"] = lambda robot: robot.line_sensors.sensor_values[i]
    
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