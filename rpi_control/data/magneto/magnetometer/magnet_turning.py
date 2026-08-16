import os, sys
script_dir = os.path.dirname(os.path.abspath(__file__))
src_dir = os.path.abspath(os.path.join(script_dir, "../"))
sys.path.append(src_dir)
from helper import *


input, output = get_filenames("magnet_turning", __file__)
columns, length = read_csv(input)
data = columns_to_dict(columns)

fig = plt.figure()
ax = fig.add_subplot()

calib_mag_orient = [float(value) * -pi/180 for value in data["calib_mag_orient"]]
ref_orient = [float(value) for value in data["ref_orient"]]
orient = [float(value) for value in data["orient"]]

t = [1/70 * i for i in range(len(calib_mag_orient))]

start = 0
for i in range(len(ref_orient)):
    if ref_orient[i] != 0:
        start = i - 1
        break
offset = calib_mag_orient[start]
offset = 1/70/3 * sum(calib_mag_orient[start - 3*70:start])
for i in range(len(calib_mag_orient)):
    calib_mag_orient[i] -= offset

offset = 0
for i in range(start + 1, len(calib_mag_orient)):
    calib_mag_orient[i] += offset
    if calib_mag_orient[i] - calib_mag_orient[i - 1] < -pi:
        offset += 2*pi
        calib_mag_orient[i] += 2*pi
    elif calib_mag_orient[i] - calib_mag_orient[i - 1] > pi:
        offset -= 2*pi
        calib_mag_orient[i] -= 2*pi

calib_mag_orient = [v / pi * 180 for v in calib_mag_orient]
ref_orient = [v / pi * 180 for v in ref_orient]
orient = [v / pi * 180 for v in orient]

ax.plot(t, calib_mag_orient, label="calib_mag_orient")
ax.plot(t, ref_orient, label="ref_orient")
ax.plot(t, orient, label="orient")

ax.set_xlabel("time [s]")
ax.set_ylabel("orient value [rad]")
plt.show()
