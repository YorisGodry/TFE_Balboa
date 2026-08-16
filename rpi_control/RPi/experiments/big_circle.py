
from importer import *
import importer

constants.strafing_impacts_theta = True
#constants.max_speed = 200



def make_controller(k_p, k_i):
    from signal_ import Signal, MovingAverage #type:ignore
    from path_tracker import ReferencePoint #type:ignore
    from position import AbsolutePosition #type:ignore
    from motors import Motors #type:ignore
    from math import cos, sin

    class NewController: # tries to make the robot follow the reference point
        def __init__(self):
            self.distance_error = Signal()
            self.speed_error = MovingAverage(5)
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
            self.speed_error.extend(self.distance_error.derivative, delta_time)
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
            if False:
                dE = self.distance_error.pid(73, 0, 59)
            else:
                dE = self.distance_error.pid(73, 0, 0) + self.speed_error.pid(59, 0, 0)
            acceleration = dE / 79.3
            acceleration = acceleration * constants.count_per_mm * constants.iteration_duration
            motors.accelerate(acceleration, acceleration)





            # orientation/theta control
            diff = self.theta_error.pid(k_p, k_i, 0) 
            diff = diff * constants.robot_width

            # strafe control
            if constants.strafing_impacts_theta:
                diff += self.strafe_error.pid(k_p, k_i, 0) * 0.5

            motors.accelerate_no_memory(diff, -diff) # add speed on this iteration only

        def reset(self):
            self.distance_error.reset()
            self.strafe_error.reset()
            self.theta_error.reset()
            self.alpha_error.reset()

    return NewController()



robot = create_robot(104, False, False)
#set_lin_reg_motors(robot)

if False:
    # simple rotation
    rotation = 2*pi
    from path_tracker import ReferencePointBehavior, Rotate #type:ignore
    behavior = ReferencePointBehavior(Rotate(0, rotation))
else:
    # follow a circle
    size = 400
    rotation = 6*pi
    from path_tracker import Circle, ReferencePointMovement, ReferencePointBehavior, create_forward_profile #type:ignore
    circle_path = Circle(0, size, size, -pi/2, rotation)
    circle = ReferencePointMovement(circle_path, create_forward_profile, True)
    behavior = ReferencePointBehavior(circle)

behavior.controller = make_controller(2, 0)

set_columns("left_command,right_command,left_speed,right_speed,x,y,alpha,theta,ref_x,ref_y,ref_alpha,ref_theta")
gather["ref_x"] = lambda robot: behavior.ref_point.x
gather["ref_y"] = lambda robot: behavior.ref_point.y
gather["ref_alpha"] = lambda robot: behavior.ref_point.alpha/pi*180
gather["ref_theta"] = lambda robot: behavior.ref_point.theta/pi*180

importer.show_compute_time = True
main(robot, behavior)