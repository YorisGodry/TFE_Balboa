import os, sys
script_dir = os.path.dirname(os.path.abspath(__file__))
src_dir = os.path.abspath(os.path.join(script_dir, "../core"))
sys.path.append(src_dir)

from signal_ import Signal # type: ignore
from position import AbsolutePosition # type: ignore
from motors import Motors # type: ignore
import constants # type: ignore
from speed_profiles import ConstantJerk, ConstantAcceleration, ConstantSpeed, ConstantAccelerationInfiniteDistance # type: ignore
from utils import find_closest # type: ignore

from math import pi, cos, sin, sqrt, atan2
from main import Behavior



### --- Start of generic ReferencePointBehavior structure ---


class ReferencePoint: # data representing the wanted state of the robot
    def __init__(self):
        self.x, self.y, self.theta, self.alpha = 0, 0, 0, 0
    def reset(self):
        self.__init__()

class ReferencePointPlanner: # abstract class
    def __init__(self):
        pass
    def update(self, ref_point, delta_time):
        pass
    def is_finished(self):
        pass

class ReferencePointController: # tries to make the robot follow the reference point
    def __init__(self):
        self.distance_error = Signal()
        self.orientation_error = Signal()

    def update(self, ref_point: ReferencePoint, position: AbsolutePosition, angle: Signal, motors: Motors, delta_time):
        # errors
        pos_x, pos_y, pos_theta = position.get_relative()
        x_error = pos_x - ref_point.x
        y_error = pos_y - ref_point.y
        orientation_error = pos_theta - ref_point.theta
        theta = ref_point.theta
        distance_error = x_error * cos(-theta) + y_error * cos(-theta + pi / 2)
        strafe_error = x_error * sin(-theta) + y_error * sin(-theta + pi / 2)
        
        # inclinaison control
        angle_prediction = angle.pid(1, 0, 0.2) - ref_point.alpha 
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

class ReferencePointBehavior(Behavior):
    def __init__(self, planner):
        self.ref_point = ReferencePoint()
        self.planner = planner
        self.controller = ReferencePointController()
    def update(self, robot, delta_time):
        self.planner.update(self.ref_point, delta_time)
        self.controller.update(self.ref_point, robot.position, robot.angle, robot.motors, delta_time)
    def reset(self):
        self.ref_point.reset()
        self.planner.reset()
        self.controller.reset()


### --- End of generic ReferencePointBehavior structure ---



### --- Start of specific implementation of ReferencePoint planners ---


forward_smoothness = 2
rotation_smoothness = 1

def set_forward_smoothness(value):
    global forward_smoothness
    assert 1 <= value <= 3
    forward_smoothness = value

def set_rotation_smoothness(value):
    global rotation_smoothness
    assert 1 <= value <= 2
    rotation_smoothness = value

def create_forward_profile(distance):
    MS, MA, MJ = constants.max_speed, constants.max_acceleration, constants.max_jerk
    if forward_smoothness == 1:
        return ConstantSpeed(distance, MS)
    elif forward_smoothness == 2:
        return ConstantAcceleration(distance, MS, MA)
    elif forward_smoothness == 3:
        return ConstantJerk(distance, MS, MA, MJ)

def create_rotation_profile(delta_theta):
    MS, MA = constants.max_rotational_speed, constants.max_rotational_acceleration
    if rotation_smoothness == 1:
        return ConstantSpeed(delta_theta, MS)
    elif rotation_smoothness == 2:
        return ConstantAcceleration(delta_theta, MS, MA)


class Delay(ReferencePointPlanner):
    def __init__(self, delay):
        self.delay = delay
        self.time = 0
    def update(self, ref_point, delta_time):
        self.time += delta_time
    def is_finished(self):
        return self.time >= self.delay

class Forward(ReferencePointPlanner):
    def __init__(self, start_x, start_y, end_x, end_y):
        self.start_x, self.start_y = start_x, start_y
        self.end_x, self.end_y = end_x, end_y
        self.dx, self.dy = end_x - start_x, end_y - start_y
        distance = sqrt(self.dx * self.dx + self.dy * self.dy)
        self.profile = create_forward_profile(distance)
        self.time = 0
    def update(self, ref_point, delta_time): 
        ## TODO: modify refpoint.alpha based on acceleration
        self.time += delta_time
        ratio = self.profile.poll_ratio(self.time)
        ref_point.x = self.start_x + self.dx * ratio
        ref_point.y = self.start_y + self.dy * ratio
    def is_finished(self):
        return self.time >= self.profile.total_time

class Rotate(ReferencePointPlanner):
    def __init__(self, start_theta, end_theta):
        self.start_theta = start_theta
        end_theta = find_closest(start_theta, end_theta, 2 * pi)
        self.d_theta = end_theta - start_theta
        self.profile = create_rotation_profile(abs(self.d_theta))
        self.time = 0
    def update(self, ref_point, delta_time):
        self.time += delta_time
        ratio = self.profile.poll_ratio(self.time)
        ref_point.theta = self.start_theta + self.d_theta * ratio
    def is_finished(self):
        return self.time >= self.profile.total_time

class Aim(ReferencePointPlanner):
    def __init__(self, start_x, start_y, start_theta, end_x, end_y):
        end_theta = atan2(end_y - start_y, end_x - start_x)
        end_theta = find_closest(start_theta, end_theta, 2 * pi)
        self.planner = Rotate(start_theta, end_theta)
    def update(self, ref_point, delta_time):
        self.planner.update(ref_point, delta_time)
    def is_finished(self):
        return self.planner.is_finished()

class OneSegment(ReferencePointPlanner):
    def __init__(self, end_x, end_y, end_theta):
        self.end_x, self.end_y, self.end_theta = end_x, end_y, end_theta
        self.planner = None
        self.step_index = -1
    def update(self, ref_point, delta_time):

        # go to next step if necessary
        # if (haven't started yet) or (finished the previous step):
        if self.step_index == -1 or (self.planner is not None and self.planner.is_finished()):
            self.step_index += 1
            
            # create next step/planner
            if self.step_index == 0:
                self.planner = Aim(ref_point.x, ref_point.y, ref_point.theta, self.end_x, self.end_y)
            elif self.step_index == 1:
                self.planner = Delay(constants.after_rotate_delay)
            elif self.step_index == 2:
                self.planner = Forward(ref_point.x, ref_point.y, self.end_x, self.end_y)
            elif self.step_index == 3:
                self.planner = Delay(constants.after_forward_delay)
            elif self.step_index == 4 and self.end_theta is not None:
                self.planner = Rotate(ref_point.theta, self.end_theta)
            elif self.step_index == 5 and self.end_theta is not None:
                self.planner = Delay(constants.after_rotate_delay)
            else:
                self.planner = None

        # update the planner
        if self.planner is not None:
            self.planner.update(ref_point, delta_time)

    def is_finished(self):
        return self.step_index != -1 and self.planner is None

class PathTracker(ReferencePointPlanner):
    def __init__(self, position_goals, theta_goals, path_repeat):
        self.index = -1
        self.position_goals = position_goals
        self.theta_goals = theta_goals
        if self.theta_goals is None:
            self.theta_goals = [None for i in range(len(self.position_goals))]
        assert len(self.theta_goals) == len(self.position_goals)
        self.path_repeat = path_repeat
        self.segment = None
    def reset(self):
        self.segment = None
        self.index = -1
    def update(self, ref_point, delta_time):
        
        # create new segment if necessary
        if self.index == -1 or (self.segment is not None and self.segment.is_finished()):
            self.index += 1
            if self.path_repeat:
                self.index %= len(self.position_goals)
            
            if self.index >= len(self.position_goals):
                self.segment = None
            else:
                x, y = self.position_goals[self.index]
                theta = self.theta_goals[self.index]
                self.segment = OneSegment(x, y, theta)

                # for testing:
                print("added one segment to", x, y, theta)

        # update current segment
        if self.segment is not None:
            self.segment.update(ref_point, delta_time)

    def is_finished(self):
        return self.index != -1 and self.segment is None

class AbsolutePathTracker(PathTracker):
    def __init__(self, robot_position, position_goals, theta_goals, path_repeat):
        self.offset_applied = False
        self.init_position_goals = position_goals
        self.robot_position = robot_position
        super().__init__(position_goals, theta_goals, path_repeat)
    def reset(self):
        self.offset_applied = False
        super().reset()
    def update(self, ref_point, delta_time):
        if not self.offset_applied:
            offset_x = -self.robot_position.init_x
            offset_y = -self.robot_position.init_y
            self.position_goals = [(x - offset_x, y - offset_y) for x, y in self.init_position_goals]
            super().reset()
        super().update(ref_point, delta_time)

class InfiniteCircle(ReferencePointPlanner):
    def __init__(self, center_x, center_y, start_angle, radius):
        self.center_x = center_x
        self.center_y = center_y
        self.start_angle = start_angle
        self.radius = radius
        self.profile = ConstantAccelerationInfiniteDistance(constants.max_speed, constants.max_acceleration)
        self.time = 0
    def update(self, ref_point, delta_time):
        self.time += delta_time
        angle = self.start_angle + self.profile.poll(self.time) / self.radius
        ref_point.theta = angle
        ref_point.x = self.center_x + sin(angle) * self.radius
        ref_point.y = self.center_y - cos(angle) * self.radius
    def reset(self):
        self.time = 0
    def is_finished(self):
        return False