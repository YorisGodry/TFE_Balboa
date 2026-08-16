import os, sys
script_dir = os.path.dirname(os.path.abspath(__file__))
src_dir = os.path.abspath(os.path.join(script_dir, "../"))
sys.path.append(src_dir)
from helper import *






max_repeat_number = 20


fig = plt.figure(figsize=(9, 9/216*155))
fig.subplots_adjust(left=0.04, bottom=0.06, right=1.0, top=0.95, wspace=0.0, hspace=0.145)
avg_fig = plt.figure(figsize=(9, 9/216*155))
avg_fig.subplots_adjust(left=0.04, bottom=0.06, right=1.0, top=0.95, wspace=0.0, hspace=0.145)

for filename_index in range(4):

    def prep(fig, filename_index):
        ax = fig.add_subplot(2, 2, filename_index + 1)
        ax.set_xlim(0, 216)
        ax.set_ylim(0, 155)
        ax.set_aspect("equal")
        if filename_index == 0:
            ax.set_title("Not using DWM")
            ax.set_ylabel("Not using magnetometer")
        if filename_index == 1:
            ax.set_title("Using DWM")
        if filename_index == 2:
            ax.set_ylabel("Using magnetometer")
        color, alpha = 0.8, 0.4
        ax.vlines([84, 84 + 40], [0, 0], [155, 155], (color, color, color), alpha=alpha)
        ax.hlines([52, 52 + 40], [0, 0], [216, 216], (color, color, color), alpha=alpha)
        return ax
    ax = prep(fig, filename_index)

    filename = "20"
    if filename_index % 2:
        filename += "_dwm"
    if filename_index // 2:
        filename += "_mag"
    #print(filename)

    try:
        input, output = get_filenames(filename, __file__)
        columns, length = read_csv(input)
        data = columns_to_dict(columns)
    except Exception:
        continue

    def fd(name, f):
        return list(map(f, data[name]))
    def fds(names, f):
        return tuple([fd(name, f) for name in names.split(",")])
    abs_x, abs_y, abs_theta = fds("abs_x,abs_y,abs_theta", float)
    rel_x, rel_y, rel_theta = fds("x,y,theta", float)
    dwm_x, dwm_y, mag_theta = fds("latest_dwm_x,latest_dwm_y,mag_theta", float) 
    repeat_number = fd("repeat_number", int)

    def split(values, numbers, max):
        splits = []
        for i in range(len(numbers)):
            if numbers[i] == -1:
                continue
            if numbers[i] == len(splits) - 1:
                splits[numbers[i]].append(values[i])
                continue
            if numbers[i] == len(splits):
                if len(splits) == max:
                    return splits
                if len(splits) > 0:
                    average = (values[i] + values[i - 1]) / 2
                    splits[-1].append(average)
                    splits.append([average, values[i]])
                else:
                    splits.append([values[i]])
        assert Exception("Not enough repeats")

    abs_x = split(abs_x, repeat_number, max_repeat_number)
    abs_y = split(abs_y, repeat_number, max_repeat_number)
    abs_theta = split(abs_theta, repeat_number, max_repeat_number)
    rel_x = split(rel_x, repeat_number, max_repeat_number)
    rel_y = split(rel_y, repeat_number, max_repeat_number)
    rel_theta = split(rel_theta, repeat_number, max_repeat_number)
    dwm_x = split(dwm_x, repeat_number, max_repeat_number)
    dwm_y = split(dwm_y, repeat_number, max_repeat_number)
    mag_theta = split(mag_theta, repeat_number, max_repeat_number)

    def get_robot_position(rel_theta, dwm_x, dwm_y):
        theta = rel_theta
        x, y = (0, -50)
        rotx, roty = cos(theta) * x - sin(theta) * y, sin(theta) * x + cos(theta) * y
        return dwm_x - rotx, dwm_y - roty

    def unique(xs, ys):
        indices = []
        previous_x, previous_y = None, None
        for i in range(1, len(xs) - 1):
            if xs[i] != previous_x or ys[i] != previous_y:
                indices.append(i)
                previous_x = xs[i]
                previous_y = ys[i]
        return indices

    offset_dwm_x, offset_dwm_y = [], []
    for lt, lx, ly in zip(rel_theta, dwm_x, dwm_y):
        indices = unique(lx, ly)
        offset_dwm_x.append([])
        offset_dwm_y.append([])
        for i in indices:
            t, x, y = lt[i], lx[i], ly[i]
            x, y = get_robot_position(t, x, y)
            offset_dwm_x[-1].append(x)
            offset_dwm_y[-1].append(y)
    

    scaled = lambda l: [v/10 for v in l]

    color_names = {0: "dark blue", max_repeat_number//2 - 1: "green", max_repeat_number - 1: "red"}
    for i in range(max_repeat_number):
        color = COLORS[color_names.get(i, "orange")]
        alpha = 1.0 if i in color_names.keys() else 0.2
        scatter_alpha = 0.3 if i in color_names.keys() else 0.0

        ax.plot(scaled(abs_x[i]), scaled(abs_y[i]), color=color, alpha=alpha)
        ax.scatter(scaled(offset_dwm_x[i]), scaled(offset_dwm_y[i]), color=color, alpha=scatter_alpha)

    






    def avg(l):
        return sum(l) / len(l)
    print(filename)
    offset_dwm_x = list(map(avg, offset_dwm_x))
    offset_dwm_y = list(map(avg, offset_dwm_y))
    abs_x = list(map(avg, abs_x))
    abs_y = list(map(avg, abs_y))

    avg_ax = prep(avg_fig, filename_index)

    avg_ax.plot(scaled(offset_dwm_x), scaled(offset_dwm_y), label="avg pos seen by dwm")
    avg_ax.plot(scaled(abs_x), scaled(abs_y), label="avg pos seen by robot")


fig.savefig("drift_20.svg")
plt.show()