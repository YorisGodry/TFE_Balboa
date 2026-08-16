
from math import sqrt, pi, atan2
from utils import find_closest


def cross(a, b):
    ax, ay, az = a
    bx, by, bz = b
    x = ay * bz - az * by
    y = az * bx - ax * bz
    z = ax * by - ay * bx
    return (x, y, z)

def dot(a, b):
    ax, ay, az = a
    bx, by, bz = b
    return ax * bx + ay * by + az * bz

def normalize(vector):
    length = sqrt(dot(vector, vector))
    x, y, z = vector
    return (x / length, y / length, z / length)


# values from magneto.c

# previous values, short test
#B = [-2285.18,  210.75, -643.10]
#Ainv = [[  0.27293,  0.01334, -0.04531], 
#        [  0.01334,  0.33141,  0.00728], 
#        [ -0.04531,  0.00728,  0.33801]]

# new values, longer test
#B = [-2036.95,  -79.01, -519.58]
#Ainv = [[  0.27414,  0.01477, -0.04715],
#        [  0.01477,  0.33221,  0.00965],
#        [ -0.04715,  0.00965,  0.33362]]

# newer values, test with dwm, big difference in biases, I don't know why
B = [  705.55,  599.22, 1458.78] # Hm= 1000
Ainv = [[  0.27075,  0.01323, -0.04376],
        [  0.01323,  0.33302,  0.00718],
        [ -0.04376,  0.00718,  0.33302]]


B = [  817.47, -698.01, 1807.01]
Ainv = [[  0.26625,  0.01564, -0.04782],
        [  0.01564,  0.32702,  0.00798],
        [ -0.04782,  0.00798,  0.32892]]

B = [  437.58, -230.87, 1407.47]
Ainv = [[  0.26173,  0.01347, -0.04462],
        [  0.01347,  0.31893,  0.00632],
        [ -0.04462,  0.00632,  0.32506]]

B = [ -436.73, -661.06,  611.28]
Ainv = [[  0.27351,  0.01367, -0.04730],
        [  0.01367,  0.33075,  0.01372],
        [ -0.04730,  0.01372,  0.33308]]


B = [ -672.22, -614.01,  819.61]
Ainv = [[  0.27261,  0.01095, -0.04723],
        [  0.01095,  0.32568,  0.01730],
        [ -0.04723,  0.01730,  0.35805]]





class Magnetometer:
    def __init__(self, lis3mdl, lsm6, calibration_time):
        self.lis3mdl = lis3mdl
        self.lsm6 = lsm6
        self.heading = (0, 0, -1) # gives orientation of front of robot when standing up

        self.calibration_sum = 0
        self.calibration_count = 0
        self.calibration_time_needed = calibration_time
        self.calibration_time = 0
    
    def reset(self):
        self.calibration_sum = 0
        self.calibration_count = 0
        self.calibration_time = 0

    def get_orientation(self): # [rad]
        self.lis3mdl.read()
        m = (self.lis3mdl.m.x, self.lis3mdl.m.y, self.lis3mdl.m.z)
        self.lsm6.read_accel()
        a = (self.lsm6.a.x, self.lsm6.a.y, self.lsm6.a.z)

        # scale raw m based on calibration values found by magneto.c
        x = m[0] - B[0]
        y = m[1] - B[1]
        z = m[2] - B[2]
        mx = Ainv[0][0] * x + Ainv[0][1] * y + Ainv[0][2] * z
        my = Ainv[1][0] * x + Ainv[1][1] * y + Ainv[1][2] * z
        mz = Ainv[2][0] * x + Ainv[2][1] * y + Ainv[2][2] * z
        m = (mx, my, mz)

        # D X M = E, cross acceleration vector Down with M (magnetic north + inclination) to produce "East"
        east = cross(m, a); # Balboa: acc vector is Up when horizontal
        east = normalize(east);

        # E X D = N, cross "East" with "Down" to produce "North" (parallel to the ground)
        north = cross(a, east); # on Balboa: Up x East
        north = normalize(north);

        # compute heading, get Y and X components of heading from E dot p and N dot p
        orientation = atan2(dot(east, self.heading), dot(north, self.heading));
        #if orientation < 0:
        #    orientation += 2*pi
        return -orientation # '-' because positive = CW for magnetometer, but position = CCW for Position

    def calibrate(self, delta_time):
        self.calibration_time += delta_time
        if self.calibration_count == 0:
            current = 0
        else:
            current = self.get_calibrated_offset()
        self.calibration_sum += find_closest(current, self.get_orientation(), 2 * pi)
        self.calibration_count += 1
    
    def is_initialized(self):
        return self.calibration_time >= self.calibration_time_needed

    def get_calibrated_offset(self):
        return self.calibration_sum / self.calibration_count