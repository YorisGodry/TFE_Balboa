import matplotlib.pyplot as plt
import numpy as np
from math import pi, sin, cos

def read_csv(filename):
    with open(filename, "r") as file:
        columns = [(column, []) for column in file.readline().strip().split(",")]
        for line in file.readlines():
            line = line.strip().split(",")
            for (name, data), value in zip(columns, line):
                data.append(value)
    return columns, len(columns[0][1])

def map(l, f):
    return [f(v) for v in l]

def str_to_bool(s):
    return s == "True"
            
base_filename = "line_following"
input_filename = base_filename + ".csv"
output_filename = __file__ + "_" + base_filename + ".svg"
from os.path import dirname, abspath, join
dir_name = dirname(abspath(__file__))
input_filename = abspath(join(dir_name, input_filename))
output_filename = abspath(join(dir_name, output_filename))

def columns_to_dict(columns):
    d = {}
    for name, data in columns:
        d[name] = data
    return d

columns, length = read_csv(input_filename)
data = columns_to_dict(columns)
hz = 70

from random import random

fig, axes = plt.subplots(1, 5)
fig.suptitle("Sensor values when line following")

for i, color in enumerate(["red", "orange", "yellow", "green", "blue"]):
    ax = axes[i]
    sensor = f"s{i}"
    angles = []
    values = []
    for angle, value in zip(data["angle"], data[sensor]):
        if int(value) == 0:
            continue
        angles.append(float(angle))
        values.append(int(value))
    ax.scatter(angles, values, s=2, alpha=0.2, c=color, label=sensor)
    ax.set_xlim(-35, 35)
    ax.set_ylim(0, 2500)
    ax.grid(True)
    if i == 2:
        ax.set_xlabel("balancing angle [°]")
    if i == 0:
        ax.set_ylabel("reflectance measured [? - from 0 to 2500]")
plt.legend()
plt.savefig(output_filename)
plt.show()
