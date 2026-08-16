import matplotlib.pyplot as plt
from math import pi, sin, cos
import sys

import os, sys
script_dir = os.path.dirname(os.path.abspath(__file__))
core_dir = os.path.abspath(os.path.join(script_dir, "../RPi/core/"))
sys.path.append(core_dir)


color_values = [(0.9, 0.6, 0.0),
          (0.35, 0.7, 0.9),
          (0.0, 0.6, 0.5),
          (0.95, 0.9, 0.25),
          (0.0, 0.45, 0.7),
          (0.8, 0.4, 0.0),
          (0.8, 0.6, 0.7),
          (0.0, 0.0, 0.0)]
color_names = ["orange", "light blue", "green", "yellow", "dark blue", "red", "pink", "black"]
COLORS = {}
for i, name in enumerate(color_names):
    COLORS[i] = color_values[i]
    COLORS[color_names[i]] = color_values[i]


def get_arg():
    assert len(sys.argv) == 2, "Expecting filename as argument"
    return sys.argv[1]

def get_filenames(base, script):
    input = base + ".csv"
    output = script + "_" + base + "NEW.pdf"
    from os.path import dirname, abspath, join
    directory = dirname(abspath(script))
    input = abspath(join(directory, input))
    output = abspath(join(directory, output))
    return input, output

def read_csv(filename):
    with open(filename, "r") as file:
        columns = [(column.strip(), []) for column in file.readline().strip().split(",")]
        for line in file.readlines():
            line = line.strip().split(",")
            for (name, data), value in zip(columns, line):
                data.append(value.strip())
    return columns, len(columns[0][1])

def columns_to_dict(columns):
    d = {}
    for name, data in columns:
        d[name] = data
    return d

def rotate(x, y, theta):
    # (x, y) rotated by theta = x * (1,0) rotated by theta + y * (0,1) rotated by theta
    rx = cos(theta) * x + cos(theta + pi/2) * y
    ry = sin(theta) * x + sin(theta + pi/2) * y
    return (rx, ry)

def unzip(l):
    result = [[] for _ in range(len(l[0]))]
    for t in l:
        for i in range(len(result)):
            result[i].append(t[i])
    return tuple(result)

def mm_to_cm(l):
    return [v / 10 for v in l]

def clamp(value, m, M):
    return m if value < m else M if value > M else value

"""
import os, sys
script_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.abspath(os.path.join(script_dir, "../"))
sys.path.append(parent_dir)
import helper
"""