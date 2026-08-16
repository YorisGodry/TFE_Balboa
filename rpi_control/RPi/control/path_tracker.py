import os, sys
script_dir = os.path.dirname(os.path.abspath(__file__))
src_dir = os.path.abspath(os.path.join(script_dir, "../core"))
sys.path.append(src_dir)

from signal_ import Signal # type: ignore
from position import AbsolutePosition # type: ignore
from motors import Motors # type: ignore
import constants # type: ignore
from speed_profiles import *
from utils import find_closest, clamp, sign # type: ignore

from math import pi, cos, sin, sqrt, atan2
from behavior import Behavior



### --- Start of generic ReferencePointBehavior structure ---


class ReferencePoint: 
    """ 
    Data representing the wanted state of the robot
    See ReferencePointBehavior
    """
    def __init__(self):
        self.x, self.y, self.theta, self.alpha = 0, 0, 0, constants.alpha_offset
    def reset(self):
        self.__init__()
    def set_alpha_from_acceleration(self, acceleration):
        ratio = acceleration / constants.max_acceleration
        self.alpha = constants.max_ref_alpha * ratio + constants.alpha_offset

class ReferencePointPlanner: # abstract class
    """
    Class that modifies a ReferencePoint
    See ReferencePointBehavior

    A Planner can create other Planners or a Path to delegate the work to
    """
    def __init__(self):
        raise Exception
    def reset(self):
        raise Exception
    def update(self, ref_point, delta_time):
        raise Exception
    def is_finished(self):
        raise Exception

class EmptyReferencePointPlanner(ReferencePointPlanner):
    def __init__(self):
        pass
    def reset(self):
        pass
    def update(self, ref_point, delta_time):
        pass
    def is_finished(self):
        return False

class Path: # abstract class
    """
    Directly modifies a ReferencePoint based on a distance along the path

    See ReferencePointMovement
    """
    def __init__(self, on_repeat=False):
        self.length = 0
        self.on_repeat = on_repeat
        raise Exception
    def update(self, ref_point, distance):
        if self.on_repeat:
            distance = distance % self.length
        else:
            distance = clamp(distance, 0, self.length)
        # update refpoint.[x, y][theta]
        raise Exception

class ReferencePointMovement(ReferencePointPlanner):
    def __init__(self, path, create_profile_function, affects_alpha):
        # if the profile is infinite and the path is on repeat, the movement will be on repeat
        self.path = path
        self.profile = create_profile_function(path.length)
        self.time = 0
        self.affects_alpha = affects_alpha
    def reset(self):
        self.time = 0
    def update(self, ref_point, delta_time):
        self.time += delta_time
        acceleration = self.profile.poll_acceleration(self.time)
        if self.affects_alpha:
            ref_point.set_alpha_from_acceleration(acceleration)
        distance = self.profile.poll_distance(self.time)
        self.path.update(ref_point, distance)
    def is_finished(self):
        return self.profile.is_finished(self.time)

class ReferencePointController: # tries to make the robot follow the reference point
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

        # orientation/theta control
        diff = self.theta_error.pid(4, 3, 0) 
        diff = diff * constants.robot_width
        motors.accelerate_no_memory(diff, -diff) # add speed on this iteration only

        # strafe control
        if constants.strafing_impacts_theta:
            diff = self.strafe_error.pid(4, 3, 0) * 0.5
            motors.accelerate_no_memory(diff, -diff) # add speed on this iteration only
        # strafe is tricky: 
        #   if the robot is always moving, then simply turning the robot
        #       to go where it is suppposed to go is enough
        #   if the robot is stationary, slowly making one of the wheels
        #       more sensitive (with a small integral constant) will help 
        #       bring the robot back where it is supposed to be
        #   but turning the robot when it is stationary will not help
        #   and making the wheels more sensitive if the robot is planning
        #       to move forward has to be done carefully so it doesn't 
        #       run off too fast, or turns too wildly (I have not tested
        #       this option)
        #   Coding motors that adapt and switch modes from sensitivity to 
        #       turning based on whether the robot is moving or not
        #   'making the wheels more sensitive' is like doing a motors
        #       calibration dynamically by increasing the voltage in one
        #       of the wheels (see core/motors.py:LinRegMotors
        #       for an example of motor calibration)
    
    def reset(self):
        self.distance_error.reset()
        self.strafe_error.reset()
        self.theta_error.reset()
        self.alpha_error.reset()

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
    
def Balancer():
    return ReferencePointBehavior(EmptyReferencePointPlanner())


### --- End of generic ReferencePointBehavior structure ---


forward_smoothness = 3
rotation_smoothness = 2

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

def create_infinite_forward_profile(distance):
    MS, MA, MJ = constants.max_speed, constants.max_acceleration, constants.max_jerk
    if forward_smoothness == 1:
        return ConstantSpeedInfiniteDistance(MS)
    elif forward_smoothness == 2:
        return ConstantAccelerationInfiniteDistance(MS, MA)
    elif forward_smoothness == 3:
        return ConstantJerkInfiniteDistance(MS, MA, MJ)

def create_rotation_profile(delta_theta):
    MS, MA = constants.max_rotational_speed, constants.max_rotational_acceleration
    if rotation_smoothness == 1:
        return ConstantSpeed(delta_theta, MS)
    elif rotation_smoothness == 2:
        return ConstantAcceleration(delta_theta, MS, MA)

def create_infinite_rotation_profile(delta_theta):
    MS, MA = constants.max_rotational_speed, constants.max_rotational_acceleration
    if rotation_smoothness == 1:
        return ConstantSpeedInfiniteDistance(MS)
    elif rotation_smoothness == 2:
        return ConstantAccelerationInfiniteDistance(MS, MA)


### --- Start of specific implementation of ReferencePoint planners ---


class Delay(ReferencePointPlanner):
    def __init__(self, delay):
        self.delay = delay
        self.time = 0
    def reset(self):
        self.time = 0
    def update(self, ref_point, delta_time):
        self.time += delta_time
    def is_finished(self):
        return self.time >= self.delay


class Line(Path):
    def __init__(self, start_x, start_y, end_x, end_y):
        self.start_x, self.start_y = start_x, start_y
        self.end_x, self.end_y = end_x, end_y
        self.dx, self.dy = end_x - start_x, end_y - start_y
        self.length = sqrt(self.dx * self.dx + self.dy * self.dy)
    def update(self, ref_point, distance):
        if self.length > 0:
            ratio = distance / self.length
        else:
            ratio = 1
        ratio = clamp(ratio, 0, 1)
        ref_point.x = self.start_x + self.dx * ratio
        ref_point.y = self.start_y + self.dy * ratio

def Forward(start_x, start_y, end_x, end_y):
    path = Line(start_x, start_y, end_x, end_y)
    return ReferencePointMovement(path, create_forward_profile, True)


class Rotation(Path):
    def __init__(self, start_theta, delta_theta):
        self.start_theta = start_theta
        self.length = abs(delta_theta)
        self.direction_of_turning = sign(delta_theta)
    def update(self, ref_point, distance):
        ref_point.theta = self.start_theta + self.direction_of_turning * distance

def Rotate(start_theta, delta_theta):
    path = Rotation(start_theta, delta_theta)
    return ReferencePointMovement(path, create_rotation_profile, False)

def InfiniteRotation(start_theta, direction_of_turning):
    path = Rotation(start_theta, sign(direction_of_turning) * 2 * pi)
    return ReferencePointMovement(path, create_infinite_rotation_profile, False)

def Turn(start_theta, end_theta):
    end_theta = find_closest(start_theta, end_theta, 2 * pi)
    path = Rotation(start_theta, end_theta - start_theta)
    return ReferencePointMovement(path, create_rotation_profile, False)

def Aim(start_x, start_y, start_theta, end_x, end_y):
    end_theta = atan2(end_y - start_y, end_x - start_x)
    return Turn(start_theta, end_theta)


class SegmentedPlanner(ReferencePointPlanner):
    def __init__(self, planner_makers, on_repeat):
        self.planner_makers = planner_makers
        self.planner = None
        self.index = -1
        self.on_repeat = on_repeat
        self.repeat_number = -1
    def reset(self):
        self.planner = None
        self.index = -1
    def update(self, ref_point, delta_time):
        # go to next step if necessary
        # if (haven't started yet) or (finished the previous step):
        if self.index == -1 or (self.planner is not None and self.planner.is_finished()):
            self.index += 1
            if self.repeat_number == -1:
                self.repeat_number = 0
            if self.on_repeat:
                if self.index == len(self.planner_makers):
                    self.index = 0
                    self.repeat_number += 1
            # create next step/planner
            if self.index < len(self.planner_makers):
                self.planner = self.planner_makers[self.index](ref_point)
            else:
                self.planner = None
        # update the planner
        if self.planner is not None:
            self.planner.update(ref_point, delta_time)
    def is_finished(self):
        return self.index != -1 and self.planner is None

def OneSegment(end_x, end_y, end_theta):
    planner_makers = [
        lambda ref_point: Aim(ref_point.x, ref_point.y, ref_point.theta, end_x, end_y),
        lambda ref_point: Delay(constants.after_rotate_delay),
        lambda ref_point: Forward(ref_point.x, ref_point.y, end_x, end_y),
        lambda ref_point: Delay(constants.after_forward_delay)
    ]
    if end_theta is not None:
        planner_makers.append(lambda ref_point: Turn(ref_point.theta, end_theta))
        planner_makers.append(lambda ref_point: Delay(constants.after_rotate_delay))
    return SegmentedPlanner(planner_makers, False)

def PathTracker(position_goals, theta_goals, on_repeat):
    if theta_goals is None:
        theta_goals = [None for i in range(len(position_goals))]
    assert len(theta_goals) == len(position_goals)
    planner_makers = []
    for (x, y), theta in zip(position_goals, theta_goals):
        # x, y and theta change after lambda function declaration
        # -> only the last value will be rememebered
        # -> need to bind x, y and theta
        # (lazy instantiation is weirder than I thought...)
        f = lambda x, y, theta: lambda ref_point: OneSegment(x, y, theta)
        planner_makers.append(f(x, y, theta))
    return SegmentedPlanner(planner_makers, on_repeat)
























class SegmentedPath(Path):
    def __init__(self, paths, on_repeat=False):
        self.paths = paths
        self.length = sum([path.length for path in paths])
        self.on_repeat = on_repeat
    def update(self, ref_point, distance):
        if self.on_repeat:
            distance = distance % self.length
        else:
            distance = clamp(distance, 0, self.length)
        for path in self.paths:
            if distance <= path.length:
                path.update(ref_point, distance)
                return
            distance -= path.length
        self.paths[-1].update(ref_point, distance)

class ParametricCurvePath(Path):
    """ ratio in [0, 1] """
    def __init__(self):
        self.length = self.compute_length()
        #opti that i talk about in update
        self.previous_ratio = 0
        self.previous_distance = 0
    def compute_length(self):
        return self.travel(0)[1]
    def sample(self, ratio):
        raise NotImplementedError("This is an abstract method")
        x, y = 0, 0
        return x, y
    def update(self, ref_point, distance):
        # possible optimisation: save the last ratio and distance traveled to only travel along the latest segment
        # this could be necessary to make large parametric curve computable in real-time
        # here, I just recompute everything from scratch, and I'll keep it as long as it works
        # (also the precision might not need to be that high)
        # (most of this class only works for 'well-behaved' parametric curves)
        # (and you could make 'travel_segment' travel in segment of size <= segment_size instead of = segment_size)
        if distance > self.length - 0.001:
            return
        if self.previous_distance < distance:
            distance -= self.previous_distance
        else:
            self.previous_ratio = 0
            self.previous_distance = 0
        delta_ratio, _ = self.travel(self.previous_ratio, distance)
        ratio = self.previous_ratio + delta_ratio
        x0, y0 = self.sample(ratio)
        ref_point.x = x0
        ref_point.y = y0
        x1, y1 = self.sample(ratio + 0.00001)
        theta = atan2(y1 - y0, x1 - x0)
        ref_point.theta = find_closest(ref_point.theta, theta, 2 * pi)
        self.previous_distance += distance
        self.previous_ratio = ratio
    def sample_delta(self, ratio, delta):
        x0, y0 = self.sample(ratio)
        x1, y1 = self.sample(ratio + delta)
        dx = x1 - x0
        dy = y1 - y0
        return sqrt(dx**2 + dy**2)
    def sample_speed(self, ratio, delta=0.00001):
        return self.sample_delta(ratio, delta) / delta
    def travel_segment(self, ratio, distance, precision=0.001):
        speed = self.sample_speed(ratio)
        guess_delta = distance / speed
        if ratio + guess_delta > 1 - precision:
            return 1 - ratio, self.sample_delta(ratio, 1 - ratio)
        guess_distance = self.sample_delta(ratio, guess_delta)
        while abs(guess_distance - distance) > precision:
            guess_delta = guess_delta / guess_distance * distance
            if ratio + guess_delta > 1 - precision:
                return 1 - ratio, self.sample_delta(ratio, 1 - ratio)
            guess_distance = self.sample_delta(ratio, guess_delta)
        return guess_delta, guess_distance
    def travel(self, ratio, distance=None, segment_size=1, precision=0.001):
        """
        start at ratio
        travel distance (or stop at ratio = 1)
            in segments of size in range [segment_size +/- precision]
        return delta <= 1 - ratio, total distance traveled <= distance
        """
        def min(a, b):
            if a is None:
                return b
            if b is None:
                return a
            if a < b:
                return a
            return b
        total_distance_traveled = 0
        total_delta = 0
        while distance is None or total_distance_traveled < distance - precision:
            current_ratio = ratio + total_delta
            current_distance_left = None if distance is None else distance - total_distance_traveled
            delta, traveled = self.travel_segment(current_ratio, min(current_distance_left, segment_size), precision)
            total_distance_traveled += traveled
            total_delta += delta
            if current_ratio + delta >= 1 - precision:
                return 1 - ratio, total_distance_traveled
        return total_delta, total_distance_traveled

class Circle(ParametricCurvePath):
    def __init__(self, center_x, center_y, radius, start_theta, delta_theta):
        self.center_x, self.center_y, self.radius = center_x, center_y, radius
        self.start_theta, self.delta_theta = start_theta, delta_theta

        self.previous_ratio, self.previous_distance = 0, 0

        super().__init__()
    def sample(self, ratio):
        theta = self.start_theta + ratio * self.delta_theta
        x = self.center_x + self.radius * cos(theta)
        y = self.center_y + self.radius * sin(theta)
        return x, y
    
class Rose(ParametricCurvePath):
    def __init__(self, size):
        self.size = size
        super().__init__()
    def sample(self, ratio):
        theta = ratio * 2 * pi
        x = cos(2 * theta) * cos(theta) * self.size
        y = cos(2 * theta) * sin(theta) * self.size
        return x, y

class Note(ParametricCurvePath):
    def __init__(self, size):
        self.size = size
        super().__init__()
    def sample(self, ratio):
        theta = ratio * 2 * pi
        values = [  (1, (-84.35448, -34.33566)),
                    (-1, (40.46448, 27.38368)),
                    (2, (-2.437736, -11.34379)),
                    (-2, (-0.964393, -4.067661)),
                    (3, (12.36604, 24.28837)),
                    (-3, (18.5965, 11.26359)),
                    (4, (-5.285697, 9.192366)),
                    (-4, (2.235886, 6.051937)),
                    (5, (-8.28655, 4.917418)),
                    (-5, (2.84373, 4.221581)),
                    (6, (2.652406, 1.709839)),
                    (-6, (1.016227, 5.090219)),
                    (7, (1.862628, 2.86866)),
                    (-7, (0.254709, 0.096835)),
                    (8, (0.909513, 1.553532)),
                    (-8, (-0.146985, 1.061961)),
                    (9, (-0.404105, 0.369253)),
                    (-9, (0.00649, 0.891086)),
                    (10, (0.262302, 0.718354)),
                    (-10, (0.642596, 0.985032))]
        x, y = 0, 0
        for freq, (init_x, init_y) in values:
            x += init_x * cos(freq * theta) + init_y * cos(freq * theta + pi/2)
            y += init_x * sin(freq * theta) + init_y * sin(freq * theta + pi/2)

        minx, miny, maxx, maxy = -117.65524270829054, -136.9302530686856, 104.94786995660854, 166.7723776563223
        width, height = maxx - minx, maxy - miny
        centerx, centery = minx + width / 2, miny + height / 2
        
        x -= centerx
        y -= centery
        size = max(width, height) / 2
        x = x / size * self.size
        y = y / size * self.size

        x, y = -y, -x # axes for building the shape and axes of robot are different

        return x, y


















"""
class Circle(ReferencePointPlanner):
    def __init__(self, radius):
        self.radius = radius
        self.center_set = False
        self.center_x, self.center_y = None, None
    def reset(self):
        self.center_set = False
        self.center_x, self.center_y = None, None
    def update(self, ref_point, delta_time):
        if not self.center_set:

    def is_finished(self):
        raise Exception
"""





# TODO curved PT using splines, iterative algo like parametriccurve, 
# would need to compute its length first, then run robot (path precomputation step)




#TODO add [Infinite]Circle, [Infinite]Rose as function that create a ParametricCurve
#   (possibly in a way that looks like Rotation with Rotate,Turn,Aim)

"""
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
"""




"""
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
""" 
# TODO: add a MakeAbsolute function that makes the AbsolutePosition.relative = zero, and changes the refpoint accordingly
# simply call that function at the start of custombehavior before proceding as usual
# MakeAbsolute would then need to be a Planner that first calls the function, then does the custombehavior

