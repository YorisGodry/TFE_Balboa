from signal_ import BoundQueueSum
from math import sin, cos
import constants
from time import time


class DWM:
    """
    Read the decawave information from the balboa buffer...
    """
    def __init__(self, balboa):
        self.balboa
        self.x, self.y = 0, 0
        self.previous_position = None
    
    def reset(self):
        self.x, self.y = 0, 0
        self.previous_position = None
    
    def is_filled(self):
        return self.previous_position is not None

    def read(self):
        data = self.balboa.read_uwb()
        distance, x, y, z = data
        position = (x, y)
        if position != self.previous_position:
            self.previous_position = position
            self.x, self.y = x, y
    
    def get_robot_position(self, position):
        _, _, theta = position.get_relative()
        x, y = constants.dwm_position
        rotx, roty = cos(theta) * x - sin(theta) * y, sin(theta) * x + cos(theta) * y
        return self.x - rotx, self.y - roty


class MovingAverageDWM(DWM):
    def __init__(self, balboa, window_size):
        self.balboa = balboa
        self.xs = BoundQueueSum(window_size)
        self.ys = BoundQueueSum(window_size)
        self.x, self.y = 0, 0
        self.previous_position = None

        self.previous_timestamp = None
    
    def reset(self):
        self.xs.reset()
        self.ys.reset()
        self.x, self.y = 0, 0
        self.previous_position = None

        self.previous_timestamp = None

    def is_filled(self):
        return self.xs.is_full()
    
    def read(self):
        data = self.balboa.read_uwb()
        distance, x, y, z = data
        position = (x, y)
        if position != self.previous_position:
            self.previous_position = position
            self.xs.push(x)
            self.ys.push(y)
            self.x, self.y = self.xs.average(), self.ys.average()
            
            self.previous_timestamp = time()
        else:
            if self.previous_timestamp is None:
                self.previous_timestamp = time()
            t = time() - self.previous_timestamp
            if t >= 0.25:
                print("!Warning! --- Not received dwm information in", round(t, 3), "seconds")
























"""
* Master's Thesis *
Implementation of a robotic swarm platform
based on the Balboa self-balancing robot
© 2025 Romain Englebert
"""

"""def hex_str(data):
    return " ".join(f"{b:02X}" for b in data)"""

import numpy as np

"""def error(err_code):
    if err_code == 0:
        print("OK")
    elif err_code == 1:
        print("unknown command or broken TLV frame")
    elif err_code == 2:
        print("internal error")
    elif err_code == 3:
        print("invalid parameter")
    elif err_code == 4:
        print("busy")
    elif err_code == 5:
        print("operation not permitted")"""


class RemoveOutliersDWM:
    """ This is Romain's code (slightly modified), I'm not using it at the moment """

    def __init__(self, balboa, window_size=5, verbose=False):

        # Use self.dwm_loc_get() if DWM1001 is connected to the RPi
        self.rocky = balboa

        self.distances = []
        self.positions = []
        self.distance = 0
        self.position = 0

        self.WINDOW = window_size  # Number of measures kept for moving average filtering
        self.VERBOSE = verbose


    def read(self):
        """
        Only use when DWM1001 is connected on the Balboa UART port, use self.dwm_loc_get() if it is connected to the RPi UART port
        """

        data = self.rocky.read_uwb()  # Read values from Balboa via i2c
        #print(data)
        # Fill memory buffer for postprocessing
        self.distances.append(data[0]/10)
        self.positions.append(list(np.array(data[1:3])/10))

        # Update current distance and position
        self.distance = data[0]/10
        self.position = list(np.array(data[1:3])/10)


    def postprocess(self, calibration: bool):
        """
        Remove outliers and use an approximate linear model of the sensor for distance measurements.

        To get a model of the sensor (a and b), use ../utils/dwm_calibration.py
        """

        if len(self.distances) > self.WINDOW:
            del self.distances[0]
            del self.positions[0]

        # Filter distances
        distances = np.array(self.distances)
        Q1 = np.percentile(distances, 40)
        Q3 = np.percentile(distances, 60)
        IQR = Q3 - Q1
        lower_bound = Q1 - 1.5 * IQR
        upper_bound = Q3 + 1.5 * IQR
        distances = distances[(distances >= lower_bound) & (distances <= upper_bound)]
        d = np.mean(distances)

        self.distance = d

        # Filter position
        pos_meas = np.array(self.positions)
        mean_meas = np.mean(pos_meas, axis=0)
        pos_dist = np.linalg.norm(pos_meas - mean_meas, axis=1)
        Q1 = np.percentile(pos_dist, 40)
        Q3 = np.percentile(pos_dist, 60)
        IQR = Q3 - Q1
        bound = Q3 + 1.5 * IQR
        pos_meas = pos_meas[pos_dist <= bound] #yoris: why not also remove using lowerbound?
        pos_meas = np.mean(pos_meas, axis=0)

        self.position = pos_meas

        if calibration:

            # Calibrate distances
            a = 0.9469764536578285
            b = -5.96484910568
            self.distance = a * d + b

            # Calibrate position
            A = np.array([np.array([ 0.93538932,  0.00353817]),
                        np.array([-0.01071457,  0.94470965])])
            b = np.array([4.6239576, 5.0861534])

            self.position = (A @ pos_meas.T).T + b