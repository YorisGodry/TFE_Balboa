import os, sys
script_dir = os.path.dirname(os.path.abspath(__file__))
src_dir = os.path.abspath(os.path.join(script_dir, "../core"))
sys.path.append(src_dir)

from signal_ import Signal # type: ignore
from position import AbsolutePosition # type: ignore
from motors import Motors # type: ignore
import constants # type: ignore
from planner import ConstantAcceleration, ConstantSpeed # type: ignore
from utils import find_closest # type: ignore

from math import pi, cos, sin, sqrt, atan2
from main import Behavior


class Forward:
    def __init__(self, start_x, start_y, end_x, end_y):
        self.start_x, self.start_y = start_x, start_y
        self.end_x, self.end_y = end_x, end_y
        dx = end_x - start_x
        dy = end_y - start_y
        self.dx, self.dy = dx, dy
        distance = sqrt(dx*dx + dy*dy)
        self.profile = ConstantAcceleration(distance, constants.max_speed, constants.max_acc)
    def poll(self, time):
        ratio = self.profile.poll_ratio(time)
        x = self.start_x + self.dx * ratio
        y = self.start_y + self.dy * ratio
        return x, y
    def is_finished(self, time):
        return time >= self.profile.total_time

class Rotate:
    def __init__(self, start_theta, end_theta):
        self.start_theta = start_theta
        end_theta = find_closest(start_theta, end_theta, 2 * pi)
        self.d_theta = end_theta - start_theta
        self.profile = ConstantSpeed(abs(self.d_theta), constants.max_rotational_speed)
    def poll(self, time):
        ratio = self.profile.poll_ratio(time)
        return self.start_theta + self.d_theta * ratio
    def is_finished(self, time):
        return time >= self.profile.total_time

class Aim:
    def __init__(self, start_x, start_y, start_theta, end_x, end_y):
        self.start_theta = start_theta
        end_theta = atan2(end_y - start_y, end_x - start_x)
        end_theta = find_closest(start_theta, end_theta, 2 * pi)
        self.d_theta = end_theta - start_theta
        self.profile = ConstantSpeed(abs(self.d_theta), constants.max_rotational_speed)
    def poll(self, time):
        ratio = self.profile.poll_ratio(time)
        return self.start_theta + self.d_theta * ratio
    def is_finished(self, time):
        return time >= self.profile.total_time

class Planner:
    DELAY = 1
    AIM = 2
    GO = 3
    ROTATE = 4

    def __init__(self, position_goals, theta_goals, path_repeat):
        self.ref_x = Signal() # mm
        self.ref_y = Signal() # mm
        self.ref_theta = Signal() # rad
        self.state = self.DELAY
        self.next_state = self.AIM
        self.delay_time = constants.init_delay_time
        self.current_goal_index = 1

        self.state_object = None
        self.time = 0

        self.position_goals = position_goals
        self.theta_goals = theta_goals
        self.path_repeat = path_repeat

    def reset(self):
        self.__init__(self.position_goals, self.theta_goals, self.path_repeat)
    
    def get_position_goal(self):
        x_goal = self.position_goals[self.current_goal_index][0]
        y_goal = self.position_goals[self.current_goal_index][1]
        return x_goal, y_goal    
        
    def start_state(self):
        if self.state == self.DELAY:
            pass
        elif self.state == self.AIM:
            x_goal, y_goal = self.get_position_goal()
            self.state_object = Aim(self.ref_x.value, self.ref_y.value, self.ref_theta.value, x_goal, y_goal)
        elif self.state == self.GO:
            x_goal, y_goal = self.get_position_goal()
            self.state_object = Forward(self.ref_x.value, self.ref_y.value, x_goal, y_goal)
        elif self.state == self.ROTATE:
            theta_goal = self.theta_goals[self.current_goal_index]
            self.state_object = Rotate(self.ref_theta.value, theta_goal)
        self.time = 0

    def start_next_state(self):
        if self.state == self.DELAY:
            self.state = self.next_state
            self.next_state = self.DELAY
        elif self.state == self.AIM:
            self.state = self.DELAY
            self.next_state = self.GO
            self.delay_time = constants.aim_to_go_delay
        elif self.state == self.GO:
            self.state = self.DELAY
            self.next_state = self.ROTATE
            self.delay_time = constants.go_to_rotate_delay
        elif self.state == self.ROTATE:
            self.state = self.DELAY
            self.next_state = self.AIM
            self.delay_time = constants.rotate_to_aim_delay

            self.current_goal_index += 1
            if self.current_goal_index >= len(self.position_goals):
                if self.path_repeat:
                    self.current_goal_index = 0
                else:
                    self.next_state = self.DELAY
        self.start_state()
            
    def update_delay(self, delta_time):
        self.ref_x.extend(self.ref_x.value, delta_time)
        self.ref_y.extend(self.ref_y.value, delta_time)
        self.ref_theta.extend(self.ref_theta.value, delta_time)

        self.delay_time -= constants.iteration_duration
        if self.delay_time <= 0:
            self.start_next_state()
        
    def rotate(self, delta_time):
        theta = self.state_object.poll(self.time)
        self.ref_theta.extend(theta, delta_time)

        if self.state_object.is_finished(self.time):
            self.start_next_state()

    def go_forward(self, delta_time):
        x, y = self.state_object.poll(self.time)
        self.ref_x.extend(x, delta_time)
        self.ref_y.extend(y, delta_time)

        if self.state_object.is_finished(self.time):
            self.start_next_state()

    def update(self, delta_time):
        ### the first time this function has been called:
        ### refx.value = position.x -> would turn the relative PT to an absolute PT (i think)
        self.time += delta_time
        if self.state == self.DELAY:
            self.update_delay(delta_time)
        elif self.state == self.AIM:
            self.rotate(delta_time)
        elif self.state == self.GO:
            self.go_forward(delta_time)
        elif self.state == self.ROTATE:
            self.rotate(delta_time)


class Controller:
    def __init__(self):
        self.distance_error = Signal()
        self.orientation_error = Signal()

    def update(self, planner: Planner, position: AbsolutePosition, angle: Signal, motors: Motors, delta_time):
        """ 
        Compute the errors using these signals:
        - planner.ref_x/ref_y/ref_theta
        - position.x/y/theta
        - angle
        
        And uses motors.accelerate(_no_memory) for control
        [req reset]
        """
        # errors
        pos_x, pos_y, pos_theta = position.get_relative()
        x_error = pos_x - planner.ref_x.value
        y_error = pos_y - planner.ref_y.value
        orientation_error = pos_theta - planner.ref_theta.value
        theta = planner.ref_theta.value
        distance_error = x_error * cos(-theta) + y_error * cos(-theta + pi / 2)
        strafe_error = x_error * sin(-theta) + y_error * sin(-theta + pi / 2)
        
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

class PathTracker(Behavior):
    # could add an absolute position mode in which the first point is
    #   set at the position where the robot is on startup -> fetch from robot.position
    #   when PT behavior starts (see comment in Planner.update)
    def __init__(self, position_goals, theta_goals, path_repeat):
        self.planner = Planner(position_goals, theta_goals, path_repeat)
        self.controller = Controller()
    def reset(self):
        self.planner.reset()
        self.controller.reset()
    def update(self, robot, delta_time):
        self.planner.update(delta_time)
        self.controller.update(self.planner, robot.position, robot.angle, robot.motors, delta_time)
