import os, sys
script_dir = os.path.dirname(os.path.abspath(__file__))
src_dir = os.path.abspath(os.path.join(script_dir, "../core"))
sys.path.append(src_dir)

from signal_ import Signal # type: ignore
from position import Position # type: ignore
from motors import Motors # type: ignore
import constants # type: ignore

#from main import Behavior


class Controller:
    def __init__(self):
        self.distance_error = Signal()
        self.orientation_error = Signal()

    def update(self, position, angle: Signal, motors: Motors, delta_time):
        """ 
        Compute the errors using these signals:
        - planner.ref_x/ref_y/ref_theta
        - position.x/y/theta
        - angle
        
        And uses motors.accelerate(_no_memory) for control
        [req reset]
        """
        # errors
        x_error, y_error, orientation_error = position.get_relative()
        distance_error = x_error
        strafe_error = y_error
        
        # inclinaison control
        angle_prediction = angle.pid(1, 0, 0.2)
        acceleration = angle_prediction * 3974.6 * constants.iteration_duration
        motors.accelerate(acceleration, acceleration)

        # orientation control
        error = orientation_error * constants.robot_width + strafe_error * 0.5
        self.orientation_error.extend(error, delta_time)
        diff = self.orientation_error.pid(2, 5, 0)
        motors.accelerate_no_memory(diff, -diff)

        # distance/speed control
        self.distance_error.extend(distance_error, delta_time)
        acceleration = self.distance_error.pid(73, 51, 59) / 79.3
        acceleration = acceleration * constants.count_per_mm * constants.iteration_duration
        motors.accelerate(acceleration, acceleration)

        # wheel sensitivity control: legacy/maybe re-add later
    
    def reset(self):
        self.distance_error.reset()
        self.orientation_error.reset()

class Balancer:#(Behavior):
    def __init__(self):
        self.controller = Controller()
    def reset(self):
        self.controller.reset()
    def update(self, robot, delta_time):
        self.controller.update(robot.position, robot.angle, robot.motors, delta_time)
