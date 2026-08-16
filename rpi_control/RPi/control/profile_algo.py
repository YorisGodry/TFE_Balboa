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

