import time
from math import floor

def clamp(value, low, high):
    value = min(high, value)
    return max(low, value)

def sign(value):
    if value < 0:
        return -1
    return 1

def find_closest(current, goal, cycle):
    lower_goal = goal + floor((current - goal) / cycle) * cycle
    if abs(current - lower_goal) <= cycle / 2:
        return lower_goal
    else:
        upper_goal = lower_goal + cycle
        return upper_goal

class Clock:
    def __init__(self):
        self.start_time = time.time()
        self.wait_until_time = self.start_time
        self.previous_delay = 0
    def add_wait_time(self, time_):
        self.wait_until_time += time_
    def wait(self, time_=0):
        self.add_wait_time(time_)
        time.sleep(max(0, self.wait_until_time - time.time()))
        self.previous_delay = time.time() - self.wait_until_time