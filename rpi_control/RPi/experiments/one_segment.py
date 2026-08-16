from importer import *

import constants #type:ignore
from path_tracker import ReferencePointBehavior, Forward, set_forward_smoothness #type:ignore
from utils import Clock #type:ignore
from main import Robot #type:ignore
from motors import LinRegMotors #type:ignore


# default values
constants.max_jerk = 4000
constants.max_acceleration = 600
constants.max_speed = 400
constants.max_ref_alpha = 6 / 180 * pi # TODO: find the relation between max_acceleration and max_ref_alpha
set_forward_smoothness(3)

# test different values
constants.max_jerk = 4000
constants.max_acceleration = 600
constants.max_speed = 400
constants.max_ref_alpha = 6 / 180 * pi
set_forward_smoothness(3)

# angle value, smoothness, linregmotors
# 4, 2, False
# 5.5, 3, False
# 6, 3, True
# ..., 2, True
distance = 600 if True else 2000

def main(robot, file):
    behavior = ReferencePointBehavior(Forward(0, 0, distance, 0))
    clock = Clock()
    set_columns("phase,x,y,theta,alpha,ref_x,ref_y,ref_theta,ref_alpha")
    gather["ref_x"] = lambda robot: behavior.ref_point.x
    gather["ref_y"] = lambda robot: behavior.ref_point.y
    gather["ref_theta"] = lambda robot: behavior.ref_point.theta / pi * 180
    gather["ref_alpha"] = lambda robot: behavior.ref_point.alpha / pi * 180
    print("Robot is ready to be raised")
    while True:
        robot.loop(behavior, constants.iteration_duration)
        write_columns(file, robot)
        #print_compute_time(clock)
        clock.wait(constants.iteration_duration)

try:
    robot = Robot(104, False, False)
    robot.motors = LinRegMotors(robot.balboa)
    with open("temp.csv", "w") as file:
        main(robot, file)
except KeyboardInterrupt:
    pass
finally:
    robot.motors.reset()