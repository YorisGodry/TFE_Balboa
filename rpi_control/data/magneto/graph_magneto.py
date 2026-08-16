import os, sys
script_dir = os.path.dirname(os.path.abspath(__file__))
src_dir = os.path.abspath(os.path.join(script_dir, "../"))
sys.path.append(src_dir)
from helper import *

from math import sqrt, atan2


filename = get_arg()
input, output = get_filenames(filename, __file__)
columns, length = read_csv(input)
data = columns_to_dict(columns)

m = list(zip(map(int, data["raw_mag_x"]), map(int, data["raw_mag_y"]), map(int, data["raw_mag_z"])))
a = list(zip(map(int, data["raw_acc_x"]), map(int, data["raw_acc_y"]), map(int, data["raw_acc_z"])))
def get_column(l, i):
    return [v[i] for v in l]

fig = plt.figure()

def plot(ax, Ainv, B, calibrated, colored, corrected_mag):

    def colors(calib_m):
        def cross(a, b):
            ax, ay, az = a
            bx, by, bz = b
            x = ay * bz - az * by
            y = az * bx - ax * bz
            z = ax * by - ay * bx
            return (x, y, z)

        def dot(a, b):
            ax, ay, az = a
            bx, by, bz = b
            return ax * bx + ay * by + az * bz

        def normalize(vector):
            length = sqrt(dot(vector, vector))
            x, y, z = vector
            return (x / length, y / length, z / length)

        def orientation(m, a):
            m, a = normalize(m), normalize(a)

            # D X M = E, cross acceleration vector Down with M (magnetic north + inclination) to produce "East"
            east = cross(m, a) # Balboa: acc vector is Up when horizontal
            east = normalize(east)

            # E X D = N, cross "East" with "Down" to produce "North" (parallel to the ground)
            north = cross(a, east) # on Balboa: Up x East
            north = normalize(north)

            heading = (0, 0, -1)
            # compute heading, get Y and X components of heading from E dot p and N dot p
            orientation = atan2(dot(east, heading), dot(north, heading))
            orientation = (-orientation) % (2*pi) # '-' because positive = CW for magnetometer, but position = CCW for Position
            color = orientation / (2*pi)
            assert 0 <= color <= 1
            return color

        return [orientation(mag, acc) for mag, acc in zip(calib_m, a)]
    
    def corrected_colors(calib_m):
        c = colors(calib_m)
        raise NotImplementedError # TODO correct mag error based on tabulation
        return c

    def calibrate(m):
        x = m[0] - B[0]
        y = m[1] - B[1]
        z = m[2] - B[2]
        mx = Ainv[0][0] * x + Ainv[0][1] * y + Ainv[0][2] * z
        my = Ainv[1][0] * x + Ainv[1][1] * y + Ainv[1][2] * z
        mz = Ainv[2][0] * x + Ainv[2][1] * y + Ainv[2][2] * z
        return mx, my, mz

    calib_m = list(map(calibrate, m))
    l = calib_m if calibrated else m

    if colored:
        if corrected_mag:
            ax.scatter(get_column(l, 0), get_column(l, 1), get_column(l, 2), 
                       c=corrected_colors(calib_m), marker="o", cmap="twilight")
        else:
            ax.scatter(get_column(l, 0), get_column(l, 1), get_column(l, 2), 
                       c=colors(calib_m), marker="o", cmap="twilight")
            
    else:
        ax.scatter(get_column(l, 0), get_column(l, 1), get_column(l, 2),
                   marker="o")


#assert filename in ["nothing", "cable", "dwm", "balancing", "linregbalancing"]
#assert filename != "linregbalancing", "Not Implemented yet"

if filename == "nothing":  
    B = [  773.90,  578.07, 1569.40] # Hm = default
    Ainv = [[  0.96948,  0.04631, -0.15755],
            [  0.04631,  1.18655,  0.04100],
            [ -0.15755,  0.04100,  1.20747]]
    B = [  773.90,  578.07, 1569.40] # Hm = 1000
    Ainv = [[  0.26995,  0.01290, -0.04387],
            [  0.01290,  0.33040,  0.01142],
            [ -0.04387,  0.01142,  0.33622]]
elif filename == "cable": 
    raise NotImplementedError
elif filename in ["dwm", "balancing"]: 
    B = [  705.55,  599.22, 1458.78] # Hm = default
    Ainv = [[  0.97621,  0.04770, -0.15777],
            [  0.04770,  1.20076,  0.02590],
            [ -0.15777,  0.02590,  1.20074]]
    B = [  705.55,  599.22, 1458.78] # Hm= 1000
    Ainv = [[  0.27075,  0.01323, -0.04376],
            [  0.01323,  0.33302,  0.00718],
            [ -0.04376,  0.00718,  0.33302]]
elif filename == "test_new":    
    B = [-1002.81, -351.18,  567.40]
    Ainv = [[  0.96937,  0.04699, -0.15791],
            [  0.04699,  1.15147,  0.03938],
            [ -0.15791,  0.03938,  1.17415]]
    B = [-1002.81, -351.18,  567.40]
    Ainv = [[  0.27050,  0.01311, -0.04407],
            [  0.01311,  0.32132,  0.01099],
            [ -0.04407,  0.01099,  0.32765]]




def set_axes(ax):
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("z")
    if filename == "balancing":
        ax.set_xlim(-1250, 1250)
        ax.set_ylim(-1250, 1250)
        ax.set_zlim(-1250, 1250)

ax1 = fig.add_subplot(1, 2, 1, projection="3d")
set_axes(ax1)

ax2 = fig.add_subplot(1, 2, 2, projection="3d")
set_axes(ax2)

if filename in ["nothing", "cable", "dwm", "test_new"]:
    plot(ax1, Ainv, B, False, False, False)
    plot(ax2, Ainv, B, True, False, False)
elif filename in ["balancing"]: 
    # this is not a good way of visualizing the mag error correction
    # you might not even see anything unless the mag errors are substantial
    plot(ax1, Ainv, B, True, True, False)
    #plot(ax2, Ainv, B, True, True, True)

plt.show()
