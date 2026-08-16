import matplotlib.pyplot as plt
from math import pi, sin, cos


import os, sys
script_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.abspath(os.path.join(script_dir, "../"))
sys.path.append(parent_dir)
from helper import *


def get_filenames(base, script):
    input = base + ".csv"
    output = script + "_" + base + ".pdf"
    from os.path import dirname, abspath, join
    directory = dirname(abspath(script))
    input = abspath(join(directory, input))
    output = abspath(join(directory, output))
    return input, output

def read_csv(filename):
    with open(filename, "r") as file:
        columns = [(column, []) for column in file.readline().strip().split(",")]
        for line in file.readlines():
            line = line.strip().split(",")
            for (name, data), value in zip(columns, line):
                data.append(value)
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


input, output = get_filenames(get_arg(), __file__)
columns, length = read_csv(input)
data = columns_to_dict(columns)
#hz = 70

def unzip(l):
    result = [[] for _ in range(len(l[0]))]
    for t in l:
        for i in range(len(result)):
            result[i].append(t[i])
    return tuple(result)

def mm_to_cm(l):
    return [v / 10 for v in l]

DELAY, AIM, GO, ROTATE = 1, 2, 3, 4
linestyles = {DELAY: {"alpha":0.3, "linestyle":"solid"},
              AIM: {"linestyle":"dotted"},
              ROTATE: {"linestyle":"dotted"},
              GO: {}}
def trace_line(x_offset, y_offset, label, color):
    plt.plot([], [], c=color, label=label)
    def finish(positions, state):
        if state == 0:
            return
        style = linestyles[state]
        xs, ys = unzip(positions)
        plt.plot(mm_to_cm(xs), mm_to_cm(ys), c=color, **style)
    positions = []
    current_state = 0
    for i in range(length):
        active = bool(int(data["active"][i]))
        if not active:
            continue
        x, y = float(data["x"][i]), float(data["y"][i])
        theta = float(data["theta"][i])
        rx, ry = rotate(x_offset, y_offset, theta)
        x, y = x + rx, y + ry
        state = int(data["state"][i])
        if state != current_state:
            positions.append((x, y))
            finish(positions, current_state)
            current_state = state
            positions = []
        positions.append((x, y))
    finish(positions, current_state)
        
trace_line(0, 0, "center of robot", "blue")
trace_line(0, 50, "left wheel", "lightblue")
trace_line(0, -50, "right wheel", "lightgreen")

position_goals = [(0, 0), (1500, 0), (1500, -600), (0, -600), (0, -900), (-300, -900), (-300, -300), (900, -300)]
xs, ys = unzip(position_goals)
plt.scatter(mm_to_cm(xs), mm_to_cm(ys), c="black", s=50, alpha=0.5, label="reference points")

plt.gca().set_aspect(1.0)

plt.yticks(range(-120, 31, 30))
plt.xticks(range(-60, 181, 30))

plt.grid(True, alpha=0.2, which="minor")
plt.minorticks_on()

plt.legend()
plt.xlabel("x [cm]")
plt.ylabel("y [cm]")
#plt.title("""Position of the robot (enc. only) as it travels
#through all the reference points (path tracking)""")
plt.grid(True)
plt.savefig(output)
plt.show()
