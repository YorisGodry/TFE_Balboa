from importer import *

import constants #type:ignore
from path_tracker import Balancer, ReferencePointBehavior, Forward, SegmentedPlanner, Delay #type:ignore
from utils import Clock #type:ignore
from main import Robot #type:ignore
from motors import LinRegMotors #type:ignore




from signal_ import Signal #type:ignore
from path_tracker import ReferencePoint #type:ignore
from position import AbsolutePosition #type:ignore
from motors import Motors #type:ignore
from math import cos, sin

class OpenLoopController:
    def __init__(self):
        self.distance_error = Signal()
        self.strafe_error = Signal()
        self.theta_error = Signal()
        self.alpha_error = Signal()
        self.motor_distance = Signal()
        self.motor_error = Signal()

    def update(self, ref_point: ReferencePoint, position: AbsolutePosition, 
               angle: Signal, motors: Motors, delta_time):
        
        self.motor_error.extend(self.motor_distance.value - ref_point.x, delta_time)

        print(self.motor_distance.value, self.motor_error.value)



        angle_mult = 1
        angle_prediction_time = 0.14 # 0.2
        distance_mult = 1
        speed_mult = 1 # 0.5



        acceleration = self.motor_error.pid(73 * distance_mult, 0, 59 * speed_mult) / 79.3
        acceleration = acceleration * constants.count_per_mm * constants.iteration_duration
        #motors.accelerate(acceleration, acceleration)

        ref_alpha = ref_point.alpha - acceleration / 3974.6 / constants.iteration_duration / angle_mult
        
        # inclinaison/alpha control
        angle_prediction = angle.pid(1, 0, angle_prediction_time)
        angle_error = angle_prediction - ref_alpha
        #angle_error = angle_prediction - ref_point.alpha - acceleration / 3974.6 / constants.iteration_duration
        acceleration = angle_error * 3974.6 * constants.iteration_duration * angle_mult
        #acceleration = prevacc - acceleration
        motors.accelerate(acceleration, acceleration)


        speed = motors.left_speed + motors.left_temp
        speed = speed / 100 * 700 * 0.248
        self.motor_distance.extend(self.motor_distance.value + speed * delta_time, delta_time)
        print(self.motor_distance.value)
    
    def reset(self):
        self.distance_error.reset()
        self.strafe_error.reset()
        self.theta_error.reset()
        self.alpha_error.reset()
        self.motor_error.reset()
        self.motor_distance.reset()




constants.stabilization_phase_delay = 3
constants.max_speed = 400/4 #mm/sec
constants.max_acceleration = 600/6 #mm/sec**2
constants.max_ref_alpha = 1 / 180 * pi 
constants.max_jerk = 4000 #mm/sec**3
distance = 600 if True else 0

def main(robot, file):
    behavior = ReferencePointBehavior(SegmentedPlanner([lambda rf: Delay(3), lambda rf: Forward(0, 0, distance, 0)], False))
    #behavior = Balancer()
    behavior.controller = OpenLoopController()
    clock = Clock()
    set_columns(file, "phase,x,y,theta,alpha,ref_x,ref_y,ref_theta,ref_alpha")
    gather["ref_x"] = lambda robot: behavior.ref_point.x
    gather["ref_y"] = lambda robot: behavior.ref_point.y
    gather["ref_theta"] = lambda robot: behavior.ref_point.theta / pi * 180
    gather["ref_alpha"] = lambda robot: behavior.ref_point.alpha / pi * 180
    print("Robot is ready to be raised")
    while True:
        robot.loop(behavior, constants.iteration_duration)
        #write_columns(file, robot, True)
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