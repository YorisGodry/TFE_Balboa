import os, sys
script_dir = os.path.dirname(os.path.abspath(__file__))
src_dir = os.path.abspath(os.path.join(script_dir, "../core"))
sys.path.append(src_dir)
from balboa import Balboa # type: ignore
from lsm6 import LSM6 # type: ignore
from dwm import DWM, MovingAverageDWM # type: ignore
from signal_ import Signal, MovingAverage, ComplementaryFilter # type: ignore
from position import Position, AbsolutePosition # type: ignore
from imu import Accelerometer, Gyroscope # type: ignore
from encoders import Encoders # type: ignore
from motors import Motors # type: ignore
import constants # type: ignore
from line_sensors import LineSensors #type:ignore

from path_tracker import Balancer
from lis3mdl import LIS3MDL #type:ignore
from magnetometer import Magnetometer #type:ignore
from utils import find_closest #type:ignore
from math import pi




class Angle(Signal):
    def __init__(self, init_angle):
        super().__init__()

        window_size = max(1, 1/7 * constants.iteration_per_second)
        window_size = round(window_size)
        print(window_size)
        self.gyro_trust = pow(0.5, constants.iteration_duration)
        print(self.gyro_trust)

        self.low_pass_filter = MovingAverage(window_size)
        self.low_pass_filter.extend(init_angle, 0)
        self.complementary_filter = ComplementaryFilter()
        self.complementary_filter.reset(self.low_pass_filter.value)
        self.value = self.complementary_filter.value

        #self.previous_gyro_data = 0
        #self.previous_acc_data = init_angle

    def update(self, accelerometer, gyroscope, delta_time):
        gyro_rate = gyroscope.get_angle_rate()
        #self.previous_gyro_data = gyro_rate
        acc_angle = accelerometer.get_angle()
        #self.previous_acc_data = acc_angle
        self.low_pass_filter.extend(acc_angle, delta_time)
        self.complementary_filter.update(gyro_rate, self.low_pass_filter.value, self.gyro_trust, delta_time)
        self.extend(self.complementary_filter.value, delta_time)










class DWMPosition(AbsolutePosition):
    # dwm update
    def __init__(self, robot, dwm_active, mag_active):
        super().__init__()
        self.dwm_active = dwm_active
        if self.dwm_active:
            self.dwm = MovingAverageDWM(robot.balboa, 5)
            self.dwm_trust = 0.001
        self.mag_active = mag_active
        if self.mag_active:
            self.magnetometer = robot.magnetometer
            self.magnet_trust = 0.001
        self.magnet_previous_theta = None

    def reset(self):
        super().reset()
        if self.dwm_active:
            self.dwm.reset()
        if self.mag_active:
            self.magnetometer.reset()
        self.magnet_previous_theta = None
    
    def update_no_initialization(self, robot, delta_time):
        self.update_encoders(robot.encoders.left.delta, robot.encoders.right.delta)
    
    def is_mag_ready(self):
        return not self.mag_active or self.magnetometer.is_initialized()
    def is_dwm_ready(self):
        return not self.dwm_active or self.dwm.is_filled()

    def update(self, robot, delta_time):
        self.update_encoders(robot.encoders.left.delta, robot.encoders.right.delta)
        if self.dwm_active:
            self.dwm.read()
        
        if not self.is_initialized():
            if self.mag_active:
                self.magnetometer.calibrate(delta_time)
            
            if self.is_mag_ready() and self.is_dwm_ready():

                if self.mag_active:
                    theta = self.magnetometer.get_calibrated_offset()
                else:
                    theta = 0

                if self.dwm_active:
                    x, y = self.dwm.get_robot_position(self)
                else:
                    x, y = 0, 0
                
                self.set_absolute_offset(x, y, theta)
        else:
            # complementary filter for x,y position
            # (should replace this code with an actual ComplementaryFilter,
            #  so that I can easily put a KalmanFilter in its place)
            if self.dwm_active:
                x, y = self.dwm.get_robot_position(self)
                t = self.dwm_trust
                self.x = t * x + (1 - t) * self.x
                self.y = t * y + (1 - t) * self.y

            # complementary filter for orientation
            if self.mag_active:
                theta = robot.magnetometer.get_orientation()
                theta = find_closest(self.theta, theta, 2*pi)
                magnet_previous_theta = theta
                t = self.magnet_trust
                self.theta = t * theta + (1 - t) * self.theta

    def is_initialized(self):
        return self.absolute_offset_set










magnetometer_init_delay = 3


class Robot:
    def __init__(self, robot_width, dwm_active, mag_active):
        constants.set_robot_width(robot_width)
        # hardware interfaces
        self.balboa = Balboa()
        self.lsm6 = LSM6()
        # sensors
        self.encoders = Encoders(self.balboa)
        self.accelerometer = Accelerometer(self.lsm6)
        self.gyro = Gyroscope(self.lsm6)
        self.lis3mdl = LIS3MDL()
        self.magnetometer = Magnetometer(self.lis3mdl, self.lsm6, magnetometer_init_delay)
        self.line_sensors = LineSensors(self.balboa)
        ##self.dwm = DWM(self.balboa) # dwm is in DWMPosition
        # state
        self.angle = Angle(self.accelerometer.get_angle())
        self.position = DWMPosition(self, dwm_active, mag_active)
        self.phase = 0
        self.time = 0
        # actuators
        self.motors = Motors(self.balboa)

        self.balancer_behavior = Balancer() # used for initialization of position (dwm)
    
    def deactivate(self):
        print("Deactivation")
        self.phase = 0
        self.motors.reset()

    def activate(self):
        self.phase = 1
        self.time = 0
        self.encoders.reset()
        self.angle.integral = 0
        self.position.reset()
        self.line_sensors.reset()

    def loop(self, behavior, delta_time):
        self.angle.update(self.accelerometer, self.gyro, delta_time)
        self.encoders.update(delta_time)
        self.line_sensors.update(delta_time)

        if self.phase == 0: # laying down
            self.position.update_no_initialization(self, delta_time)
            if abs(self.angle.value) < constants.activation_angle:
                print("Robot has been raised, starting stabilization phase")
                self.activate()
                behavior.reset()
                self.balancer_behavior.reset()

        elif self.phase == 1: # stabilization phase, just got up (finding 'alpha_zero')
            self.position.update_no_initialization(self, delta_time)
            self.balancer_behavior.update(self, delta_time)

            self.time += delta_time
            if self.time >= constants.stabilization_phase_delay:
                print("Robot stabilized, starting calibration of dwm and magnetometer (if applicable)")
                self.time = 0
                self.phase = 2

            self.motors.update()
            if abs(self.angle.value) > constants.deactivation_angle:
                self.deactivate()
            
        elif self.phase == 2: # calibration when up phase (dwm, magnet)
            self.position.update(self, delta_time)
            self.balancer_behavior.update(self, delta_time)

            if self.position.is_initialized():
                print("DWM and magnetometer calibrated (if applicable), starting custom behavior updates")
                self.phase = 3

                ## TODO: this is a quick hack, it isn't good, need to add a function to interface with controller
                # (also should rewrite Behavior better)
                behavior.controller.distance_error.integral = self.balancer_behavior.controller.distance_error.integral

            self.motors.update()
            if abs(self.angle.value) > constants.deactivation_angle:
                self.deactivate()

        elif self.phase == 3: # running custom behavior
            self.position.update(self, delta_time)
            behavior.update(self, delta_time)

            self.motors.update()
            if abs(self.angle.value) > constants.deactivation_angle:
                self.deactivate()
