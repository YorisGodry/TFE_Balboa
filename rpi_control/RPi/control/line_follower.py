import os, sys
script_dir = os.path.dirname(os.path.abspath(__file__))
src_dir = os.path.abspath(os.path.join(script_dir, "../core"))
sys.path.append(src_dir)

from signal_ import Signal #type:ignore
from planner import ConstantAccelerationInfiniteDistance #type:ignore
import constants #type:ignore
from main import Behavior


class Planner:
    def __init__(self, speed, delay):
        self.ref_distance = Signal()
        self.speed_profile = ConstantAccelerationInfiniteDistance(speed, constants.max_acc)
        self.time = -delay
        self.delay = delay

    def update(self, delta_time):
        self.time += delta_time
        self.ref_distance.extend(self.speed_profile.poll(self.time), delta_time)
    
    def reset(self):
        self.ref_distance.reset()
        self.time = -self.delay


class Controller:
    def __init__(self):
        self.distance_error = Signal()
        self.orientation_error = Signal()
        #self.reference_orientation = Signal()
    def reset(self):
        self.distance_error.reset()
        self.orientation_error.reset()
        #self.reference_orientation.reset()

    def update(self, planner, line_sensors, encoders, angle, motors, delta_time):
        if planner.time > 0:
            # line control
            diff = line_sensors.line_position.pid(1, 0, 0.3) * 1.1665 # * 100/9 / 9.525
            motors.accelerate_no_memory(-diff, diff)
        else:
            # orientation control
            theta = (encoders.right.value - encoders.left.value) / constants.robot_width
            error = theta * constants.robot_width
            self.orientation_error.extend(error, delta_time)
            diff = self.orientation_error.pid(2, 5, 0)
            motors.accelerate_no_memory(diff, -diff)
        
        # distance/speed control
        distance = (encoders.right.value + encoders.left.value) / 2
        distance_error = distance - planner.ref_distance.value
        self.distance_error.extend(distance_error, delta_time)
        acceleration = self.distance_error.pid(73, 51, 59) / 79.3
        acceleration = acceleration * constants.count_per_mm * constants.iteration_duration
        motors.accelerate(acceleration, acceleration)
        
        # inclinaison control
        angle_prediction = angle.pid(1, 0, 0.2)
        acceleration = angle_prediction * 3974.6 * constants.iteration_duration
        motors.accelerate(acceleration, acceleration)

class LineFollower(Behavior):
    def __init__(self):
        self.planner = Planner(175, 3)
        self.controller = Controller()
    def reset(self):
        self.planner.reset()
        self.controller.reset()
    def update(self, robot, delta_time):
        self.planner.update(delta_time)
        self.controller.update(self.planner, robot.line_sensors, robot.encoders, robot.angle, robot.motors, delta_time)
