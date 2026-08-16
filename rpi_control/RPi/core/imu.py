
from lsm6 import LSM6
import constants
import time
from math import atan2

class Accelerometer:
    def __init__(self, lsm6: LSM6):
        self.lsm6 = lsm6

    def get_angle(self):
        self.lsm6.read_accel()
        return atan2(self.lsm6.a.z, self.lsm6.a.x)

class Gyroscope:
    def __init__(self, lsm6: LSM6):
        self.lsm6 = lsm6
        self.y_zero = 0
        self.calibrate()
    
    def calibrate(self):
        print("Waiting to calibrate gyro...")
        time.sleep(1) # wait for IMU readings to stabilize
        print("Start of gyro calibration...")
        total = 0
        for _ in range(constants.gyro_calibration_iterations):
            self.lsm6.read_gyro()
            total += self.lsm6.g.y
            time.sleep(0.001)
        self.y_zero = total / constants.gyro_calibration_iterations
        print("End of gyro calibration")
    
    def get_angle_rate(self):
        self.lsm6.read_gyro()
        return (self.lsm6.g.y - self.y_zero) * 115.4/180000