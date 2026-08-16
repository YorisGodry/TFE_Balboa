from importer import *

from behavior import EmptyBehavior #type:ignore
from path_tracker import Rotate, ReferencePointBehavior #type:ignore
import constants #type:ignore
from utils import Clock #type:ignore
from main import Robot #type:ignore
from motors import LinRegMotors, DeadMotors #type:ignore

constants.strafing_impacts_theta = False
constants.max_rotational_speed = 2*pi / 360 / 2 * 6


def main(robot, file):
    robot.motors = DeadMotors(robot.balboa)
    behavior = ReferencePointBehavior(Rotate(0, 2*pi))
    clock = Clock()
    set_columns(file, "delta_encoder,mag_orient,theta")
    gather["delta_encoder"] = lambda robot: robot.encoders.right.value - robot.encoders.left.value
    gather["mag_orient"] = lambda robot: robot.magnetometer.get_orientation() / pi * 180
    print("Robot is ready to be raised")
    while True:
        robot.loop(behavior, constants.iteration_duration)
        write_columns(file, robot, True)
        #print_compute_time(clock)
        clock.wait(constants.iteration_duration)

try:
    robot = Robot(104, False, False)
    with open("temp.csv", "w") as file:
        main(robot, file)
except KeyboardInterrupt:
    pass
finally:
    robot.motors.reset()