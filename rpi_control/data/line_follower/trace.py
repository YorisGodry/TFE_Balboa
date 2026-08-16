import matplotlib.pyplot as plt
from math import pi, sin, cos

def get_filenames(base, script):
    input = base + ".csv"
    output = script + "_" + base + ".svg"
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


input, output = get_filenames("lf_q2", __file__)
columns, length = read_csv(input)
data = columns_to_dict(columns)
#hz = 70

robot = []
left_wheel = []
right_wheel = []
line = []
for i, (active, x, y) in enumerate(zip(data["active"], data["x"], data["y"])):
    active, x, y = active == "True", float(x), float(y)
    if not active:
        continue
    robot.append((x, y))
    theta = float(data["theta"][i]) / 180 * pi
    rx, ry = rotate(0, 50, theta)
    left_wheel.append((x + rx, y + ry))
    rx, ry = rotate(0, -50, theta)
    right_wheel.append((x + rx, y + ry))
    line_position = float(data["line_position"][i]) # in [mm]
    rx, ry = rotate(0, line_position, theta)
    line.append((x + rx, y + ry))

def f(l):
    return ([x/10 for x, y in l], [y/10 for x, y in l])
plt.plot(*f(robot), c="blue", label="center of robot")
plt.plot(*f(left_wheel), c="lightblue", label="left wheel")
plt.plot(*f(right_wheel), c="lightgreen", label="right wheel")
plt.plot(*f(line), c="red", label="line")

plt.gca().set_aspect(1.0)

plt.legend()
plt.xlabel("x [cm]")
plt.ylabel("y [cm]")
plt.title("""Position of the robot (enc. only) as it follows a line marked on the floor""")
plt.grid(True)
plt.savefig(output)
plt.show()
