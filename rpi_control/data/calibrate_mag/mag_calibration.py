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
axes = fig.subplots()
plot_data = {axes: ("ccw_east", "State of the robot when balancing"),
             1: ("balancing_linregmotors", "State of the robot when balancing using LinRegMotors")}
for ax in [axes]:
    input, output = get_filenames(plot_data[ax][0], __file__)
    columns, length = read_csv(input)
    data = columns_to_dict(columns)

    delta_encoders = list(map(float, data["delta_encoder"]))
    mag_orient = list(map(float, data["mag_orient"]))

    start_iteration = 0
    t = [1/70 * (i - start_iteration) for i in range(length)]

    ax.plot(t, delta_encoders, label="delta")
    ax.plot(t, mag_orient, label="mag")

    ax.grid(True)
    ax.set_title(plot_data[ax][1])
    ax.set_xlabel("time")
    #ax.set_ylabel("")
    #ax.set_xlim(0, 15)
    #ax.set_ylim(-40, 40)
    ax.legend()
plt.show()
