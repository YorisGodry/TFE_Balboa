import os, sys
script_dir = os.path.dirname(os.path.abspath(__file__))
src_dir = os.path.abspath(os.path.join(script_dir, "../"))
sys.path.append(src_dir)
from helper import *


#def double_integral(alpha, start_iteration):
#    mean = sum(alpha[start_iteration:]) / len(alpha[start_iteration:])
#    integral = 0
#    double = 0
#    di = []
#    for value in alpha[:start_iteration]:
#        di.append(0)
#    for value in alpha[start_iteration:]:
#        integral += (value - mean) / 70
#        double += integral / 70
#        di.append(double)
#    return di


fig = plt.figure()
axes = fig.subplots(2)
plot_data = {axes[0]: ("balancing", "State of the robot when balancing"),
             axes[1]: ("balancing_linregmotors", "State of the robot when balancing using LinRegMotors")}
for ax in axes:
    input, output = get_filenames(plot_data[ax][0], __file__)
    columns, length = read_csv(input)
    data = columns_to_dict(columns)

    phase = [int(phase) for phase in data["phase"]]
    x = [float(x) for x in data["x"]]
    y = [float(y) for y in data["y"]]
    theta = [float(theta) for theta in data["theta"]]
    alpha = [float(alpha) for alpha in data["angle"]]
    dE_I = [float(dE_I) for dE_I in data["dE.I"]]

    start_iteration = 0
    for p in phase:
        if p == 3:
            break
        else:
            start_iteration += 1
    t = [1/70 * (i - start_iteration) for i in range(length)]

    ax.plot(t, x, label="position.x (forward) [mm]")
    ax.plot(t, y, label="position.y (to the left) [mm]")
    ax.plot(t, theta, label="orientation [°]")
    ax.plot(t, alpha, label="balancing angle [°]")
    #ax.plot(t, double_integral(alpha, start_iteration), label="double integral of [alpha - mean(alpha)]")
    #ax.plot(t, dE_I, label="integral of position.x [mm*s]")

    avg = lambda l: sum(l) / len(l)
    def filter(l, f):
        result = []
        for i in range(len(l)):
            if f(i):
                result.append(l[i])
        return result
    stable = lambda i: phase[i] == 3
    print(avg(filter(alpha, stable)), avg(filter(dE_I, stable)))

    ax.grid(True)
    ax.set_title(plot_data[ax][1])
    ax.set_xlabel("time [s]")
    #ax.set_ylabel("")
    ax.set_xlim(0, 15)
    ax.set_ylim(-40, 40)
    ax.legend()
plt.show()
