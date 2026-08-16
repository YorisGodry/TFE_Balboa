import os, sys
script_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.abspath(os.path.join(script_dir, "../"))
sys.path.append(parent_dir)
from helper import *


input, output = get_filenames("motors_delay", __file__)
columns, length = read_csv(input)
data = columns_to_dict(columns)

time = [1/100 * i for i in range(length)]

plt.plot(time, [int(dl) for dl in data["delta_left"]], c="orange", label="delta left")
plt.plot(time, [int(dr) for dr in data["delta_right"]], c="blue", alpha=0.5, label="delta right")

plt.legend()
plt.xlabel("time [s]")
plt.ylabel("delta [count/iteration]")
plt.title("""Speed of motors over time (to visualize motor delay if any)""")
plt.grid(True)
plt.savefig(output)
plt.show()
