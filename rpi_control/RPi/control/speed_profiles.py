import os, sys
script_dir = os.path.dirname(os.path.abspath(__file__))
src_dir = os.path.abspath(os.path.join(script_dir, "../core"))
sys.path.append(src_dir)

from math import sqrt, cbrt
from utils import clamp #type:ignore


class Profile:
    def __init__(self):
        raise Exception
    def poll_distance(self, time):
        raise Exception
    def poll_acceleration(self, time):
        raise Exception
    def is_finished(self, time):
        raise Exception


class ConstantSpeed(Profile):
    def __init__(self, distance, max_speed):
        MS, MD = max_speed, distance
        self.max_speed = MS
        self.distance = MD
    def poll_distance(self, time):
        MS, MD = self.max_speed, self.distance
        return clamp(time * MS, 0, MD)
    def poll_acceleration(self, time):
        return 0
    def is_finished(self, time):
        MS, MD = self.max_speed, self.distance
        return time >= MD / MS
    
class ConstantSpeedInfiniteDistance(Profile):
    def __init__(self, max_speed):
        self.max_speed = max_speed
    def poll_distance(self, time):
        return time * self.max_speed
    def poll_acceleration(self, time):
        return 0
    def is_finished(self, time):
        return False

class ConstantAcceleration(Profile):
    def __init__(self, distance, max_speed, max_acceleration):
        self.max_speed, self.max_acceleration = max_speed, max_acceleration
        MS, MA = self.max_speed, self.max_acceleration
        self.total_distance = distance
        if distance >= MS * MS / MA:
            self.total_time = distance / MS + MS / MA
        else:
            self.total_time = 2 * sqrt(distance / MA)
    def poll_distance(self, time):
        MS, MA = self.max_speed, self.max_acceleration
        time = clamp(time, 0, self.total_time)
        if self.total_distance >= MS * MS / MA:
            if time <= MS / MA:
                return time * time * MA / 2
            elif time <= self.total_time - MS / MA:
                return - MS * MS / 2 / MA + MS * time
            else:
                return self.total_distance - (self.total_time - time) ** 2 * MA / 2
        else:
            if time < self.total_time / 2:
                return time * time * MA / 2
            else:
                time = self.total_time - time
                return self.total_distance - time * time * MA / 2
    def poll_acceleration(self, time):
        MS, MA = self.max_speed, self.max_acceleration
        if self.total_distance >= MS * MS / MA:
            if time <= MS / MA:
                return MA
            elif time <= self.total_time - MS / MA:
                return 0
            else:
                return -MA
        else:
            if time < self.total_time / 2:
                return MA
            else:
                return -MA
    def is_finished(self, time):
        return time >= self.total_time

class ConstantAccelerationInfiniteDistance(Profile):
    def __init__(self, max_speed, max_acceleration):
        self.max_speed, self.max_acceleration = max_speed, max_acceleration
    def poll_distance(self, time):
        MS, MA = self.max_speed, self.max_acceleration
        time = max(0, time)
        if time <= MS / MA:
            return time * time * MA / 2
        else:
            return - MS * MS / 2 / MA + MS * time
    def poll_acceleration(self, time):
        MS, MA = self.max_speed, self.max_acceleration
        if time <= MS / MA:
            return MA
        else:
            return 0
    def is_finished(self, time):
        return False
























# The following functions have been coded quite differently from the previous ones
# But the idea is still the same

class ConstantJerk(Profile):
    def __init__(self, MD, MS, MA, MJ):
        self.profile = SCurveProfile(MJ, MA, MS, MD)
    def poll_distance(self, time):
        return pollpoly(self.profile.Distance, self.profile.length, time)
    def poll_acceleration(self, time):
        return pollpoly(self.profile.Acceleration, self.profile.length, time)
    def is_finished(self, time):
        return time >= self.profile.get_total_time()

# General (Segmented)Polynomial functions

def poll(segment, value):
    s = 0
    for i in range(len(segment)):
        s += segment[i] * (value ** i)
    return s

def pollpoly(poly, length, value):
    time = 0
    for i in range(len(poly)):
        segmenttime = length(i)
        if time <= value <= time + segmenttime:
            return poll(poly[i], value - time)
        time += segmenttime
    
    if value <= 0:
        return 0
    elif value >= time:
        return poll(poly[len(poly) - 1], length(len(poly) - 1))

def integral(f, length):
    g = [[] for i in range(len(f))]
    for i in range(len(f)):
        segment = f[i]
        current = g[i]
        if i == 0:
            current.append(0.0)
        else:
            current.append(poll(g[i-1], length(i-1)))
        for j in range(len(segment)):
            current.append(segment[j] / (j + 1))
        while current[-1] == 0.0 and len(current) > 1:
            current.pop()
    return g


# To compute time values of the speed profiles

class SCurveProfile:
    def __init__(self, MJ, MA, MS, MD):
        self.MJ, self.MA, self.MS, self.MD = MJ, MA, MS, MD
        self.mar, self.msr = self.get_mar_msr()
        self.a, self.b, self.c = self.get_abc()
        a, b, c = self.a, self.b, self.c
        self.length = lambda i: [a, b, a, c, a, b, a][i]
        self.Jerk = [[MJ],
                     [0],
                     [-MJ],
                     [0],
                     [-MJ],
                     [0],
                     [MJ]]
        self.Acceleration = integral(self.Jerk, self.length)
        self.Speed = integral(self.Acceleration, self.length)
        self.Distance = integral(self.Speed, self.length)

    def get_total_time(self):
        a, b, c = self.a, self.b, self.c
        return 4*a + 2*b + c

    def get_mar_msr(self):
        MJ, MA, MS, MD = self.MJ, self.MA, self.MS, self.MD
        mar, msr = False, False
        if MA**2 > MS * MJ:
            if MD**2 * MJ > 4 * MS**3:
                msr = True
        else:
            if MD * MJ**2 > 2 * MA**3:
                mar = True
                if MD > MS**2 / MA + MS * MA / MJ:
                    msr = True    
        return mar, msr

    def get_abc(self): 
        mar, msr, MJ, MA, MS, MD = self.mar, self.msr, self.MJ, self.MA, self.MS, self.MD
        if mar and msr:
            a = MA / MJ
            b = MS / MA - MA / MJ
            c = MD / MS - MS / MA - MA / MJ
        elif mar and not msr:
            a = MA / MJ
            b = (-3 * MA**2 + sqrt(MA**4 + 4 * MJ**2 * MD * MA)) / (2 * MJ * MA)
            c = 0
        elif not mar and msr:
            a = sqrt(MS / MJ)
            b = 0
            c = MD / MS - 2*sqrt(MS / MJ)
        elif not mar and not msr:
            a = cbrt(MD / (2 * MJ))
            b = 0
            c = 0
        return float(a), float(b), float(c)























    
class ConstantJerkInfiniteDistance(Profile):
    def __init__(self, MS, MA, MJ):
        self.profile = SCurveProfileInfiniteDistance(MJ, MA, MS)
    def poll_distance(self, time):
        return pollpoly_infinite_distance(self.profile.Distance, self.profile.length, time)
    def poll_acceleration(self, time):
        return pollpoly_infinite_distance(self.profile.Acceleration, self.profile.length, time)
    def is_finished(self, time):
        return False

# General (Segmented)Polynomial functions

def pollpoly_infinite_distance(poly, length, value):
    if value <= 0:
        return 0

    time = 0
    for i in range(len(poly)):
        segmenttime = length(i)
        if segmenttime is None:
            return poll(poly[i], value - time)
        if time <= value <= time + segmenttime:
            return poll(poly[i], value - time)
        time += segmenttime
    
    if value >= time:
        return poll(poly[len(poly) - 1], length(len(poly) - 1))


# To compute time values of the speed profiles

class SCurveProfileInfiniteDistance:
    def __init__(self, MJ, MA, MS):
        self.MJ, self.MA, self.MS = MJ, MA, MS
        self.mar, self.msr = self.get_mar_msr()
        self.a, self.b, self.c = self.get_abc()
        a, b, c = self.a, self.b, self.c
        self.length = lambda i: [a, b, a, c][i]
        self.Jerk = [[MJ],
                     [0],
                     [-MJ],
                     [0]]
        self.Acceleration = integral(self.Jerk, self.length)
        self.Speed = integral(self.Acceleration, self.length)
        self.Distance = integral(self.Speed, self.length)

    def get_mar_msr(self):
        MJ, MA, MS = self.MJ, self.MA, self.MS
        return MA**2 <= MS * MJ, True

    def get_abc(self): 
        mar, msr, MJ, MA, MS = self.mar, self.msr, self.MJ, self.MA, self.MS
        if mar and msr:
            a = MA / MJ
            b = MS / MA - MA / MJ
            c = None
        elif not mar and msr:
            a = sqrt(MS / MJ)
            b = 0
            c = None
        return float(a), float(b), float(c)