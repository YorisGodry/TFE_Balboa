from importer import *

import constants #type:ignore
from path_tracker import Balancer #type:ignore
from utils import Clock #type:ignore
from main import Robot #type:ignore
from motors import LinRegMotors #type:ignore


constants.stabilization_phase_delay = 0
constants.strafing_impacts_theta = False
# test different alpha_offset values
constants.alpha_offset += 0

if True: # test rotation, find good dE.I value
    from path_tracker import Delay, SegmentedPlanner, Rotate, ReferencePointBehavior #type:ignore
    from path_tracker import set_rotation_smoothness #type:ignore
    set_rotation_smoothness(2)
    planner = SegmentedPlanner([lambda rf: Delay(60), lambda rf: Rotate(0, pi)], False)
    behavior = ReferencePointBehavior(planner)
else: # test balancing, find good alpha_zero/alpha_offset value
    behavior = Balancer()

def main(robot, file):
    clock = Clock()
    set_columns(file, "phase,x,y,alpha,ref_alpha,dE_I")
    gather["dE_I"] = lambda robot: behavior.controller.distance_error.integral
    gather["ref_alpha"] = lambda robot: behavior.ref_point.alpha / pi * 180
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