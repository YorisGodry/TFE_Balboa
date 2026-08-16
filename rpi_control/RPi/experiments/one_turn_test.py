
from importer import *

from behavior import EmptyBehavior #type:ignore
from path_tracker import Balancer, ReferencePointBehavior, Rotate #type:ignore
import constants #type:ignore
from utils import Clock #type:ignore
from main import Robot #type:ignore
from motors import DeadMotors #type:ignore


# this behavior will make the robot balance and slowly turn on itself
constants.strafing_impacts_theta = False
constants.stabilization_phase_delay = 0


















from signal_ import Signal #type:ignore
from path_tracker import ReferencePoint #type:ignore
from position import AbsolutePosition #type:ignore
from motors import Motors #type:ignore
from math import cos, sin

class NewController: # tries to make the robot follow the reference point
    def __init__(self):
        self.distance_error = Signal()
        self.strafe_error = Signal()
        self.theta_error = Signal()
        self.alpha_error = Signal()

    def update(self, ref_point: ReferencePoint, position: AbsolutePosition, 
               angle: Signal, motors: Motors, delta_time):
        x, y, theta = position.get_relative()
        alpha = angle.value
        ref_x, ref_y = ref_point.x, ref_point.y
        ref_theta, ref_alpha = ref_point.theta, ref_point.alpha
        # update error signals
        x_error = x - ref_x
        y_error = y - ref_y
        t = ref_theta
        self.distance_error.extend(x_error * cos(-t) + y_error * cos(-t + pi/2), delta_time)
        self.strafe_error.extend(x_error * sin(-t) + y_error * sin(-t + pi/2), delta_time)
        self.theta_error.extend(theta - ref_theta, delta_time)
        self.alpha_error.extend(alpha - ref_alpha, delta_time)
        
        # inclinaison/alpha control
        if True:
            angle_prediction = angle.pid(1, 0, 0.2)
            angle_error = angle_prediction - ref_alpha
        else: 
            # no difference with previous computation is ref_alpha 
            #   doesn't change
            # there will probably be a BIG problem if ref_alpha 
            #   jumps in value (rectangular acceleration profile)
            # might make things better is using a trapezoidal
            #   acceleration profile with a relatively tame jerk
            angle_error = self.alpha_error.pid(1, 0, 0.2)
        acceleration = angle_error * 3974.6 * constants.iteration_duration
        motors.accelerate(acceleration, acceleration)

        # distance/speed control ## dE.k_i = 51, is what it used to be to remove SSE on dE
        acceleration = self.distance_error.pid(73, 0, 59) / 79.3
        acceleration = acceleration * constants.count_per_mm * constants.iteration_duration
        motors.accelerate(acceleration, acceleration)



        k_p = 1

        # orientation/theta control
        diff = self.theta_error.pid(k_p, 0, 0) 
        diff = diff * constants.robot_width
        motors.accelerate_no_memory(diff, -diff) # add speed on this iteration only

        # strafe control
        if constants.strafing_impacts_theta:
            diff = self.strafe_error.pid(k_p, 0, 0) * 0.5
            motors.accelerate_no_memory(diff, -diff)
    
    def reset(self):
        self.distance_error.reset()
        self.strafe_error.reset()
        self.theta_error.reset()
        self.alpha_error.reset()





from motors import LinRegMotors #type:ignore




def main(robot, file):
    from path_tracker import Delay, SegmentedPlanner #type:ignore
    planner = SegmentedPlanner([lambda x: Delay(3), lambda x: Rotate(0, pi/2)], False)
    behavior = ReferencePointBehavior(planner)
    behavior.controller = NewController()
    #robot.motors = LinRegMotors(robot.balboa)
    clock = Clock()
    set_columns("phase,x,y,theta,alpha,ref_x,ref_y,ref_theta,ref_alpha")
    gather["ref_x"] = lambda robot: behavior.ref_point.x
    gather["ref_y"] = lambda robot: behavior.ref_point.y
    gather["ref_theta"] = lambda robot: behavior.ref_point.theta / pi * 180
    gather["ref_alpha"] = lambda robot: behavior.ref_point.alpha / pi * 180
    #set_columns(file, "phase,raw_mag_x,raw_mag_y,raw_mag_z,raw_acc_x,raw_acc_y,raw_acc_z,delta_encoder,mag_orient,theta")
    #gather["delta_encoder"] = lambda robot: robot.encoders.right.value - robot.encoders.left.value
    #gather["mag_orient"] = lambda robot: robot.magnetometer.get_orientation() / pi * 180
    print("Robot is ready to be raised")
    while True:
        robot.loop(behavior, constants.iteration_duration)
        #robot.lis3mdl.read()
        write_columns(file, robot)
        #print_compute_time(clock)
        clock.wait(constants.iteration_duration)

try:
    robot = Robot(104, False, False)
    #robot.position.magnet_trust = 0
    with open("temp.csv", "w") as file:
        main(robot, file)
except KeyboardInterrupt:
    pass
finally:
    robot.motors.reset()