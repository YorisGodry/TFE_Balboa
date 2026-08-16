import os, sys
script_dir = os.path.dirname(os.path.abspath(__file__))
src_dir = os.path.abspath(os.path.join(script_dir, "../"))
sys.path.append(src_dir)
from helper import *




def read(filename):
    input, output = get_filenames(filename, __file__)
    columns, length = read_csv(input)
    data = columns_to_dict(columns)

    def fd(name):
        return list(map(float, data[name]))
    def fds(names):
        return tuple([fd(name) for name in names.split(",")])
    x, y, alpha, ref_alpha, dE_I = fds("x,y,alpha,ref_alpha,dE_I") 
    phase = list(map(int, data["phase"]))
    for start_index in range(length):
        if phase[start_index] == 3:
            break
    time = [(i - start_index) * 1/70 for i in range(length)]

    return time, x, y, [a - r for a, r in zip(alpha, ref_alpha)]

def finish(ax, title):
    ax.grid()
    ax.legend()
    ax.set_title(title)
    ax.set_xlim(0, 60)
    ax.set_ylim(-80, 80)



try:
    filename = get_arg()
    fig = plt.figure(figsize=(9, 9))
    ax = fig.add_subplot()
    time, x, y, alpha_error = read(filename)
    ax.plot(time, x, label="x = distance error", color=COLORS[0])
    ax.plot(time, y, label="y = strafing error", color=COLORS[1])
    ax.plot(time, alpha_error, label="alpha error", color=COLORS[2])
    finish(ax, "")
    ax.set_xlim(0, 120)
    plt.show()
except Exception:


    fig = plt.figure(figsize=(9, 18))
    if True:
        filenames = ["normal", "normal_reverse", "linreg", "linreg_reverse"]
        if False:
            for i in range(len(filenames)):
                filenames[i] += "_2"
        titles = {"normal": "State of the robot using the default motors",
                "linreg": "State of the robot using the calibrated motors",
                "normal_reverse": "State of the robot using the default motors\nand pointing in the opposite direction of the previous experiment",
                "linreg_reverse": "State of the robot using the calibrated motors\nand pointing in the opposite direction of the previous experiment"}


        fig = plt.figure(figsize=(9, 9))
        for index, filename in enumerate(filenames):
            ax = fig.add_subplot(2, 1, index % 2 + 1)

            time, x, y, alpha_error = read(filename)
            ax.plot(time, x, label="x = distance error", color=COLORS[0])
            ax.plot(time, y, label="y = strafing error", color=COLORS[1])
            ax.plot(time, alpha_error, label="alpha error", color=COLORS[2])
            if index >= 2:
                time, x, y, alpha_error = read(filenames[index - 2])
                ax.plot(time, x, label="x using normal motors", color=COLORS[0], alpha=0.2)
                ax.plot(time, y, label="y using normal motors", color=COLORS[1], alpha=0.2)
                ax.plot(time, alpha_error, label="alpha error using normal motors", color=COLORS[2], alpha=0.2)

            finish(ax, titles[filename])

            if index == 1 or index == 3:
                type = "normal" if index == 1 else "linreg"
                plt.savefig(type + ".svg")
            if index == 1:
                fig = plt.figure(figsize=(9, 9))


    else:
        ax = fig.add_subplot(2, 1, 1)
        show(ax, "table_turn_zero", "turning 180° without integral")
        ax.set_xlim(0, 120)

        ax = fig.add_subplot(2, 1, 2)
        show(ax, "table_turn_tenth", "turning 180° with an integral term worth a tenth of the proportional term")
        ax.set_xlim(0, 120)