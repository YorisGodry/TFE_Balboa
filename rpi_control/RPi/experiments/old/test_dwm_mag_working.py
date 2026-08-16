from importer import *
from path_tracker_v2 import PathTracker, AbsolutePathTracker, ReferencePointBehavior, InfiniteCircle #type:ignore
from main import Robot #type:ignore
from utils import Clock, find_closest #type:ignore
import constants #type:ignore


def main(robot, file):
    position_goals = [(0, 0), (300, 0), (300, 300), (0, 300)]
    theta_goals = None # [-pi/2, 0, pi/2, pi]
    planner = AbsolutePathTracker(robot.position, position_goals, theta_goals, True)
    behavior = ReferencePointBehavior(planner)

    #behavior = ReferencePointBehavior(InfiniteCircle(0, 300, 0, 300))

    clock = Clock()
    set_columns(file, "x,y,theta")
    while True:
        robot.loop(behavior, constants.iteration_duration)
        write_columns(file, robot)
        
        """
        theta = find_closest(180, gather["theta"](robot), 360)
        mag = 0
        if robot.magnetometer.calibration_count > 0:
            mag = robot.magnetometer.get_orientation() - robot.magnetometer.get_calibrated_offset()
        mag = find_closest(180, mag / pi * 180, 360)
        print(round(theta), round(mag), round(find_closest(theta, mag, 360) - theta))
        """

        clock.add_wait_time(constants.iteration_duration)
        clock.wait()

try:
    robot = Robot(104, False, True, 3)
    with open("temp.csv", "w") as file:
        main(robot, file)
except KeyboardInterrupt:
    pass
finally:
    robot.motors.reset()