import os, sys
script_dir = os.path.dirname(os.path.abspath(__file__))
src_dir = os.path.abspath(os.path.join(script_dir, "../"))
sys.path.append(src_dir)
from helper import *


filename = get_arg()
input, output = get_filenames(filename, __file__)
columns, length = read_csv(input)
data = columns_to_dict(columns)

def fd(name):
    return list(map(float, data[name]))
def fds(names):
    return tuple([fd(name) for name in names.split(",")])
x, y, theta, alpha, ref_x, ref_y, ref_theta, ref_alpha = fds("x,y,theta,alpha,ref_x,ref_y,ref_theta,ref_alpha")
phase = list(map(int, data["phase"]))
for start_index in range(length):
    if phase[start_index] == 3:
        break
time = [(i - start_index) * 1/70 for i in range(length)]

error_x = [a - r for a, r in zip(x, ref_x)]
error_y = [a - r for a, r in zip(y, ref_y)]

fig = plt.figure(figsize=(9, 9))
plt.subplots_adjust(left=.125, bottom=.05, right=.9, top=.95, wspace=.25, hspace=.3)

acc11, acc12, acc21, acc22 = None, None, None, None
for i, a in enumerate(ref_alpha):
    if a > ref_alpha[0]:
        acc11 = time[i - 1] if acc11 is None else acc11
        acc12 = time[i + 1]
    if a < ref_alpha[0]:
        acc21 = time[i - 1] if acc21 is None else acc21
        acc22 = time[i + 1]


def show(ax, actual, expected, title):
    ax.plot(time, actual, label="actual", color=COLORS[0])
    ax.plot(time, expected, label="expected", color=COLORS[1])#, alpha=0.5)
    #ax.set_title(title)
    ax.grid()
    ax.set_xlim(acc11 - 3, acc22 + 3)
    ax.set_xlabel("time [s]")

    facecolor = "0.55"
    alpha = 0.2
    ax.axvspan(acc11, acc12, facecolor=facecolor, alpha=alpha)
    ax.axvspan(acc21, acc22, facecolor=facecolor, alpha=alpha)

    if title == "x = distance":
        ax.legend()

zero = [0 for i in range(length)]

ax = fig.add_subplot(4, 2, (1, 4))
x = [v/10 for v in x]
ref_x = [v/10 for v in ref_x]
show(ax, x, ref_x, "x = distance")
ax.set_ylim(-10, ref_x[-1] + 10)
ax.set_ylabel("forward distance [cm]")

# acc control

ax = fig.add_subplot(4, 2, 5)
error_x = [v/10 for v in error_x]
show(ax, error_x, zero, "x error = distance error")
ax.set_ylim(-10, 10)
ax.set_ylabel("forward distance error [cm]")

ax = fig.add_subplot(4, 2, 7)
show(ax, alpha, ref_alpha, "alpha")
ax.set_ylim(ref_alpha[0] - 25, ref_alpha[0] + 25)
ax.set_ylabel("balancing angle[°]")

# turn control

ax = fig.add_subplot(4, 2, 6)
y = [v/10 for v in y]
ref_y = [v/10 for v in ref_y]
show(ax, y, ref_y, "y = strafe error")
ax.set_ylim(-1.2, 1.2)
ax.set_ylabel("strafing distance [cm]")

ax = fig.add_subplot(4, 2, 8)
show(ax, theta, ref_theta, "theta")
ax.set_ylim(-6, 6)
ax.set_ylabel("orientation angle [°]")

plt.savefig(filename + ".svg")