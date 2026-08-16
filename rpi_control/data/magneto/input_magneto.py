import os, sys
script_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.abspath(os.path.join(script_dir, "../"))
sys.path.append(parent_dir)
from helper import *

filename = get_arg()
input, output = get_filenames(filename, __file__)
columns, length = read_csv(input)
data = columns_to_dict(columns)

with open(filename + "_magneto_input.csv", "w") as file:
    for i in range(length):
        x = data["raw_mag_x"][i]
        y = data["raw_mag_y"][i]
        z = data["raw_mag_z"][i]
        line = f"{int(x)},{int(y)},{int(z)}\n"
        file.write(line)