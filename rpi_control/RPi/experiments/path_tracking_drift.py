from importer import *

import constants #type:ignore
constants.after_rotate_delay = 0.5 # 1
constants.after_forward_delay = 1.5 # 3
constants.stabilization_phase_delay = 3 # 6

from path_tracker import ReferencePointBehavior, PathTracker #type:ignore
from utils import Clock #type:ignore
from main import Robot #type:ignore
from motors import LinRegMotors #type:ignore





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
    




size = 400 if True else 1000
#circle = ...
square = PathTracker([(size, 0), (size, size), (0, size), (0, 0)], None, True)
#pointy_circle = ...
#two_circles = ...

planner = square

def main(robot, file):
    behavior = ReferencePointBehavior(planner)
    clock = Clock()
    set_columns(file, "phase,repeat_number,x,y,theta,abs_x,abs_y,abs_theta,latest_dwm_x,latest_dwm_y,mag_theta,ref_x,ref_y,ref_theta")
    gather["repeat_number"] = lambda robot: behavior.planner.repeat_number
    gather["latest_dwm_x"] = lambda robot: 0 if robot.position.dwm.previous_position is None else robot.position.dwm.previous_position[0]
    gather["latest_dwm_y"] = lambda robot: 0 if robot.position.dwm.previous_position is None else robot.position.dwm.previous_position[1]
    gather["mag_theta"] = lambda robot: 0 if robot.position.magnet_previous_theta is None else robot.position.magnet_previous_theta
    gather["abs_x"] = lambda robot: robot.position.correct_position.get_absolute()[0]
    gather["abs_y"] = lambda robot: robot.position.correct_position.get_absolute()[1]
    gather["abs_theta"] = lambda robot: robot.position.correct_position.get_absolute()[2]
    gather["x"] = lambda robot: robot.position.correct_position.get_relative()[0]
    gather["y"] = lambda robot: robot.position.correct_position.get_relative()[1]
    gather["theta"] = lambda robot: robot.position.correct_position.get_relative()[2]
    gather["ref_x"] = lambda robot: behavior.ref_point.x
    gather["ref_y"] = lambda robot: behavior.ref_point.y
    gather["ref_theta"] = lambda robot: behavior.ref_point.theta

    def print_pretty(columns):
        columns = columns.split(",")
        for column in columns:
            print(column, gather[column](robot))

    print("Robot is ready to be raised")

    i = 0
    while True:
        robot.loop(behavior, constants.iteration_duration)
        write_columns(file, robot, False)
        
        i+=1
        if i % (70*5) == 0:
            print(gather["repeat_number"](robot))
            i = 0
        
        #print_pretty("x,y,theta,alpha,ref_x,ref_y,ref_theta,ref_alpha")
        #print_compute_time(clock)
        clock.wait(constants.iteration_duration)

try:
    robot = Robot(104, True, True)
    robot.position = DualPosition(robot, True, True)
    #robot.motors = LinRegMotors(robot.balboa)
    #robot.position.dwm_trust = 0
    robot.position.magnet_trust = 0
    with open("temp.csv", "w") as file:
        main(robot, file)
except KeyboardInterrupt:
    pass
finally:
    robot.motors.reset()