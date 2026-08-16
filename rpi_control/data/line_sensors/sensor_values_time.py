from helper import *


input, output = get_filenames("sensing_manually", __file__)
columns, length = read_csv(input)
data = columns_to_dict(columns)
#hz = 70

time = [1/70 * i for i in range(length)]

colors = ["red", "orange", "yellow", "green", "blue"]

for i in range(5):
    values = [2500 - int(value) for value in data[f"s{i}"]]
    plt.plot(time, values, c=colors[i], label=f"s{i}")

plt.legend()
plt.xlabel("time [s]")
plt.ylabel("sensor value [?]")
plt.title("""Sensor values of the robot when moved by hand""")
plt.grid(True)
plt.savefig(output)
plt.show()

for i in range(5):
    values = [2500 - int(value) for value in data[f"s{i}"]]
    alphas = [float(angle) / pi * 180 for angle in data["angle"]]
    plt.scatter(alphas, values, c=colors[i], label=f"s{i}", s=20, alpha=0.2)

step = 0.1
xs, ys = [], []
for a in range(-200, 200 + 1):
    a = a / 10
    xs.append(a)
    thing = 1/cos(a / 180 * pi)
    ys.append(400 * (thing - 1) / 0.062 + 420)
plt.plot(xs, ys, color="black", label="TEMP")

plt.legend()
plt.xlabel("alpha [°]")
plt.ylabel("sensor value [?]")
plt.title("""Sensor values of the robot when moved by hand""")
plt.grid(True)
plt.savefig(output)
plt.show()

for i in range(5):
    alphas = [float(angle) / pi * 180 for angle in data["angle"]]
    middles = [2500 - int(value) for value in data["s2"]]
    selves = [2500 - int(value) for value in data[f"s{i}"]]

    ### THIS IS JUST A FILTER
    a, m, s = [], [], []
    for alpha, middle, self in zip(alphas, middles, selves):
        if middle == 2500 or self == 2500:
            continue
        a.append(alpha)
        m.append(middle)
        s.append(self)
    
    ratios = [self / middle for self, middle in zip(s, m)]
    plt.scatter(a, ratios, c=colors[i], label=f"s{i}", s=20, alpha=0.2)

plt.legend()
plt.xlabel("alpha [°]")
plt.ylabel("sensor value ratios [unit]")
plt.title("""Sensor value ratios of the robot when moved by hand""")
plt.grid(True)
plt.savefig(output)
plt.show()


