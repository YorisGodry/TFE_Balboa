
import os, sys
script_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.abspath(os.path.join(script_dir, "../"))
sys.path.append(parent_dir)
from helper import *


filename = get_arg()
input, output = get_filenames(filename, __file__)
columns, length = read_csv(input)
data = columns_to_dict(columns)

def fd(name):
    return list(map(float, data[name]))
def fds(names):
    return tuple([fd(name) for name in names.split(",")])



dolpf = False





def avg(l):
    return sum(l) / len(l)

def thing(right_command, right_speed, ax=None, verbose=False):
    def lpf(l):
        w = 2
        r = []
        for i in range(len(l)):
            s = 0
            for j in range(-w, w+1):
                d = i + j
                if d < 0:
                    continue
                elif d >= len(l):
                    continue
                s += l[d]
            s /= (w*2+1)
            r.append(s)
        return r
    if dolpf:
        right_speed = lpf(right_speed)
    ref_alpha = fd("ref_alpha")
    state = 0
    alpha_zero = ref_alpha[0]
    if verbose:
        print(alpha_zero)
    end_acc = 0
    start_dec = 0
    for i in range(len(ref_alpha)):
        if state == 0 and ref_alpha[i] > alpha_zero:
            state = 1
            if verbose: 
                print("start acc", i)
        elif state == 1 and ref_alpha[i] == alpha_zero:
            state = 2
            if verbose:
                print("end acc", i)
            end_acc = i
        elif state == 2 and ref_alpha[i] < alpha_zero:
            state = 3
            if verbose:
                print("start dec", i)
            start_dec = i
        elif state == 3 and ref_alpha[i] == alpha_zero:
            state = 4
            if verbose:
                print("done decelerating", i)
    delta_i = start_dec - end_acc
    w = 2
    d = round(delta_i * (w-1)/(2*w))
    start, end = end_acc + d, start_dec - d
    delay = 3
    right_command, right_speed = right_command[start:end], right_speed[start+delay:end+delay]

    avg_command, avg_speed = avg(right_command), avg(right_speed)
    if ax is not None:
        ax.scatter(right_command, right_speed)
        ax.scatter(avg(right_command), avg(right_speed), color="red")
    return avg_command, avg_speed





right_command = fd("right_command")
right_speed = fd("right_speed")

fig = plt.figure(figsize=(9, 9))
ax = fig.add_subplot(1, 2, 1)
avg_right_command, avg_right_speed = thing(right_command, right_speed, ax)

left_command = fd("left_command")
left_speed = fd("left_speed")

ax = fig.add_subplot(1, 2, 2)
avg_left_command, avg_left_speed = thing(left_command, left_speed, ax)
plt.show()

print(avg_right_command, avg_right_speed)
print(avg_left_command, avg_left_speed)