import os, sys
script_dir = os.path.dirname(os.path.abspath(__file__))
src_dir = os.path.abspath(os.path.join(script_dir, "../"))
sys.path.append(src_dir)
from helper import *


input, output = get_filenames("new_raw", __file__)
columns, length = read_csv(input)
data = columns_to_dict(columns)

fig = plt.figure()
ax = fig.add_subplot(projection="3d")

if True:
    xs = [int(value) for value in data["raw_mag_x"]]
    ys = [int(value) for value in data["raw_mag_y"]]
    zs = [int(value) for value in data["raw_mag_z"]]

    for i in range(len(xs)):
        x, y, z = xs[i], ys[i], zs[i]

        B = [-2036.95,  -79.01, -519.58]
        Ainv = [[  0.27414,  0.01477, -0.04715],
                [  0.01477,  0.33221,  0.00965],
                [ -0.04715,  0.00965,  0.33362]]
        
        m = (x, y, z)

        x = m[0] - B[0]
        y = m[1] - B[1]
        z = m[2] - B[2]
        mx = Ainv[0][0] * x + Ainv[0][1] * y + Ainv[0][2] * z
        my = Ainv[1][0] * x + Ainv[1][1] * y + Ainv[1][2] * z
        mz = Ainv[2][0] * x + Ainv[2][1] * y + Ainv[2][2] * z
        m = (mx, my, mz)

        xs[i], ys[i], zs[i] = m
else:
    x = [float(value) for value in data["mag_x"]]
    y = [float(value) for value in data["mag_y"]]
    z = [float(value) for value in data["mag_z"]]

x, y, z = xs, ys, zs
print(len(x), len(y), len(z))
print(x)
ax.scatter(x,y,z, marker="o")

ax.set_xlabel("x")
ax.set_ylabel("y")
ax.set_zlabel("z")
plt.show()
