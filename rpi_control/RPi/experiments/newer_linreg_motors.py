from importer import *

import constants #type:ignore
constants.after_rotate_delay = 0.5 # 1
constants.after_forward_delay = 1.5 # 3
constants.stabilization_phase_delay = 3 # 6

from behavior import Behavior #type:ignore
from utils import Clock #type:ignore
from main import Robot #type:ignore
from motors import LinRegMotors #type:ignore




class MotorsBehavior(Behavior):
    def __init__(self):
        self.speeds = [100, 120, 140, 160, 180, 200, -100, -120, -140, -160, -180, -200]
        self.speed_index = 0
        self.time = 0
        self.current_speed = 0
        self.just_changed = False
    def update(self, robot, delta_time):
        if self.time % 4 == 0:
            robot.motors.accelerate(-self.current_speed, -self.current_speed)
            new_speed = 0 if self.speed_index >= len(self.speeds) else self.speeds[self.speed_index]
            robot.motors.accelerate(new_speed, new_speed)
            self.time += 1
            self.current_speed = new_speed
            self.speed_index += 1
            self.just_changed = True
        else:
            self.just_changed = False
    def reset(self):
        self.speed_index = 0
        self.time = 0
        self.current_speed = 0
        self.just_changed = False


constants.set_iteration_per_second(1)

def main(robot, file):
    behavior = MotorsBehavior()
    clock = Clock()

    set_columns(file, "command,delta_left,delta_right")
    gather["command"] = lambda robot: 0 if behavior.speed_index >= len(behavior.speeds) else behavior.speeds[behavior.speed_index]
    gather["delta_left"] = lambda robot: robot.encoders.left.delta
    gather["delta_right"] = lambda robot: robot.encoders.right.delta

    print("Robot is ready to be raised")

    while True:
        robot.loop(behavior, constants.iteration_duration)
        
        if not behavior.just_changed:
            write_columns(file, robot, True)
        
        clock.wait(constants.iteration_duration)

try:
    robot = Robot(104, False, False)
    with open("temp.csv", "w") as file:
        main(robot, file)
except KeyboardInterrupt:
    pass
finally:
    robot.motors.reset()