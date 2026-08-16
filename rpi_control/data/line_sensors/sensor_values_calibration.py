from helper import *


input, output = get_filenames("sensing_manually", __file__)
columns, length = read_csv(input)
data = columns_to_dict(columns)
#hz = 70

time = [1/70 * i for i in range(length)]

colors = ["red", "orange", "yellow", "green", "blue"]

for i in range(5):
    values = [2500 - int(value) for value in data[f"s{i}"]]
    alphas = [float(angle) / pi * 180 for angle in data["angle"]]
    plt.scatter(alphas, values, c=colors[i], label=f"s{i}", s=20, alpha=0.2)


for sensor, (color, center) in enumerate(zip(colors, [0, 1, 1, 1, 0.5])):

    sensor_values = [2500 - int(value) for value in data[f"s{sensor}"]]
    alphas = [float(angle) / pi * 180 for angle in data["angle"]]
    def find_closest(a):
        min_distance = 30
        index = 0
        for i, alpha in enumerate(alphas):
            if abs(alpha - a) < min_distance:
                index = i
                min_distance = abs(alpha - a)
        return sensor_values[index]

    things = []
    values = []
    precision = 100
    for a in range(-30*precision, 30*precision + 1):
        a = a / precision
        thing = 1/cos((a - center) / 180 * pi)
        things.append(thing)
        values.append(find_closest(a))
    if False:
        newthings = [(t - 1) / (1.16 - 1) * 40 - 20 for t in things]
        newvalues = [(v - 250) / (800 - 250) * 1000 + 1500 for v in values]
        plt.scatter(newthings, newvalues, c=color, s=1, alpha=0.5)

    from sklearn.linear_model import LinearRegression
    import numpy

    def train(l):
        model = LinearRegression()
        x, y = unzip(l)
        model.fit(numpy.array(x).reshape((-1, 1)), numpy.array(y))
        return model, model.coef_[0], model.intercept_

    def predict(model, xs):
        xs = numpy.array(xs).reshape((-1, 1))
        ys = list(model.predict(xs))
        if len(ys) == 1:
            return ys[0]
        return ys

    model, a, b = train(list(zip(things, values)))
    xs = list(range(-30, 30+1))
    ys = predict(model, xs)
    #plt.plot(xs, ys, label="PREDICTION OF MODEL", color="black")

    def thing(a):
        return 1/cos((a - center) / 180 * pi)
    ys = predict(model, [thing(x) for x in xs])
    plt.plot(xs, ys, label=f"prediction of s{sensor}", c=color)

plt.legend()
plt.xlabel("alpha [°]")
plt.ylabel("sensor value [?]")
plt.title("""Sensor values of the robot when moved by hand""")
plt.grid(True)
plt.savefig(output)
plt.show()



