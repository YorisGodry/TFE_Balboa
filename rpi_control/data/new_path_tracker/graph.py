import matplotlib.pyplot as plt
from math import pi, sin, cos


import os, sys
script_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.abspath(os.path.join(script_dir, "../"))
sys.path.append(parent_dir)
from helper import *



if True:










    for i, filename in enumerate(["a_to_b_sharp", "a_to_b", "knotted_catmull_rom", "knotted_catmull_rom_linreg"]):
        fig = plt.figure(figsize=(9, 9))
        ax = fig.add_subplot(1, 1, 1)







        def rotate(x, y, theta):
            # (x, y) rotated by theta = x * (1,0) rotated by theta + y * (0,1) rotated by theta
            rx = cos(theta) * x + cos(theta + pi/2) * y
            ry = sin(theta) * x + sin(theta + pi/2) * y
            return (rx, ry)


        #filename = get_arg()
        input, output = get_filenames(filename, __file__)
        columns, length = read_csv(input)
        data = columns_to_dict(columns)


        def fd(name):
            return list(map(float, data[name]))
        def fds(names):
            return tuple([fd(name) for name in names.split(",")])
        x, y, theta = fds("x,y,theta")
        ref_x, ref_y = fds("ref_x,ref_y")






        def wheel(xs, ys, thetas, xo, yo):
            new_xs, new_ys = [], []
            for x, y, theta in zip(xs, ys, thetas):
                theta = theta / 180 * pi
                new_x = x + xo * cos(theta) + yo * cos(theta + pi/2)
                new_xs.append(new_x)
                new_y = y + xo * sin(theta) + yo * sin(theta + pi/2)
                new_ys.append(new_y)
            return new_xs, new_ys




        scale = lambda l: [v/10 for v in l]
        ax.plot(scale(x), scale(y), color=COLORS["dark blue"], label="center of robot")
        left_x, left_y = wheel(x, y, theta, 0, 52)
        ax.plot(scale(left_x), scale(left_y), color=COLORS["light blue"], label="left wheel", alpha=0.3)
        right_x, right_y = wheel(x, y, theta, 0, -52)
        ax.plot(scale(right_x), scale(right_y), color=COLORS["green"], label="right wheel", alpha=0.3)
        ax.plot(scale(ref_x), scale(ref_y), color=COLORS["red"], label="reference", linestyle="dotted")



        ax.set_aspect("equal")

        #plt.yticks(range(-120, 31, 30))
        #plt.xticks(range(-60, 181, 30))

        #plt.grid(True, alpha=0.2, which="minor")
        #plt.minorticks_on()

        if filename == "circle":
            ax.set_ylim(-10, 90)
            ax.set_xlim(-50, 50)
        elif filename == "rose":
            ax.set_ylim(-50, 50)
            ax.set_xlim(-50, 50)
        elif filename == "note":
            ax.set_ylim(-40, 40)
            ax.set_xlim(-50, 50)
        
        if filename in ["knotted_catmull_rom", "knotted_catmull_rom_linreg"]:
            points = [(2, 1), (2, 2), (0, 3), (-0.5, 1), (-1, 5), (0, 4), (1, 4), (0, 0)]
            ax.scatter([x*30 for x, y in points], [y*30 for x, y in points], color="black", alpha=0.5, s=20)

        ax.legend()
        ax.set_xlabel("x [cm]")
        ax.set_ylabel("y [cm]")
        #ax.set_title("""Position of the robot (enc. only) as it moves using
        #the reference point tracking controller""")
        #ax.grid(True)
        plt.savefig(filename + ".pdf")
    #plt.show()







elif False:


    fig = plt.figure(figsize=(9, 3))


    for i, filename in enumerate(["circle", "rose", "note"]):
        ax = fig.add_subplot(1, 3, i + 1)








        def rotate(x, y, theta):
            # (x, y) rotated by theta = x * (1,0) rotated by theta + y * (0,1) rotated by theta
            rx = cos(theta) * x + cos(theta + pi/2) * y
            ry = sin(theta) * x + sin(theta + pi/2) * y
            return (rx, ry)


        #filename = get_arg()
        input, output = get_filenames(filename, __file__)
        columns, length = read_csv(input)
        data = columns_to_dict(columns)


        def fd(name):
            return list(map(float, data[name]))
        def fds(names):
            return tuple([fd(name) for name in names.split(",")])
        x, y, theta = fds("x,y,theta")
        ref_x, ref_y = fds("ref_x,ref_y")





        def wheel(xs, ys, thetas, xo, yo):
            new_xs, new_ys = [], []
            for x, y, theta in zip(xs, ys, thetas):
                theta = theta / 180 * pi
                new_x = x + xo * cos(theta) + yo * cos(theta + pi/2)
                new_xs.append(new_x)
                new_y = y + xo * sin(theta) + yo * sin(theta + pi/2)
                new_ys.append(new_y)
            return new_xs, new_ys




        scale = lambda l: [v/10 for v in l]
        ax.plot(scale(x), scale(y), color=COLORS["dark blue"], label="center of robot")
        left_x, left_y = wheel(x, y, theta, 0, 52)
        ax.plot(scale(left_x), scale(left_y), color=COLORS["light blue"], label="left wheel", alpha=0.3)
        right_x, right_y = wheel(x, y, theta, 0, -52)
        ax.plot(scale(right_x), scale(right_y), color=COLORS["green"], label="right wheel", alpha=0.3)
        ax.plot(scale(ref_x), scale(ref_y), color=COLORS["red"], label="reference", linestyle="dotted")



        ax.set_aspect("equal")

        #plt.yticks(range(-120, 31, 30))
        #plt.xticks(range(-60, 181, 30))

        #plt.grid(True, alpha=0.2, which="minor")
        #plt.minorticks_on()

        if filename == "circle":
            ax.set_ylim(-10, 90)
            ax.set_xlim(-50, 50)
        elif filename == "rose":
            ax.set_ylim(-50, 50)
            ax.set_xlim(-50, 50)
        elif filename == "note":
            ax.set_ylim(-40, 40)
            ax.set_xlim(-50, 50)

        ax.legend()
        ax.set_xlabel("x [cm]")
        ax.set_ylabel("y [cm]")
        #ax.set_title("""Position of the robot (enc. only) as it moves using
        #the reference point tracking controller""")
        #ax.grid(True)
    plt.savefig("all.svg")
    plt.show()
