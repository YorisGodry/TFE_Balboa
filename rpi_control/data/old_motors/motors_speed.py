import os, sys
script_dir = os.path.dirname(os.path.abspath(__file__))
src_dir = os.path.abspath(os.path.join(script_dir, "../"))
sys.path.append(src_dir)
from helper import *

input, output = get_filenames("motors_speed", __file__)
columns, length = read_csv(input)
data = columns_to_dict(columns)

plt.scatter([int(c) for c in data["command"]], [int(dl) for dl in data["delta_left"]],
            c="orange", alpha=0.8, label="left speed")
plt.scatter([int(c) for c in data["command"]], [int(dr) for dr in data["delta_right"]],
            c="blue", alpha=0.4, label="right speed")

from sklearn.linear_model import LinearRegression
import numpy
pos_left = []
pos_right = []
neg_left = []
neg_right = []
for i in range(length):
    command = int(data["command"][i])
    left, right = int(data["delta_left"][i]), int(data["delta_right"][i])
    if command > 0:
        pos_left.append((command, left))
        pos_right.append((command, right))
    elif command < 0:
        neg_left.append((command, left))
        neg_right.append((command, right))

def train(l):
    model = LinearRegression()
    x, y = unzip(l)
    model.fit(numpy.array(x).reshape((-1, 1)), numpy.array(y))
    return model, model.coef_[0], model.intercept_

def predict(model, xs):
    xs = numpy.array(xs).reshape((-1, 1))
    ys = list(model.predict(xs))
    if len(ys) == 1:
        return ys[0]
    return ys

## train on motor data
lpmodel, alp, blp = train(pos_left)
rpmodel, arp, brp = train(pos_right)
lnmodel, aln, bln = train(neg_left)
rnmodel, arn, brn = train(neg_right)

def motors(lc, rc):
    if lc > 0:
        left = predict(lpmodel, lc)
    elif lc < 0:
        left = predict(lnmodel, lc)
    else:
        left = 0
    if rc > 0:
        right = predict(rpmodel, rc)
    elif rc < 0:
        right = predict(rnmodel, rc)
    else:
        right = 0
    return left, right

## show predicted motor behavior
cs = list(range(-300, 300+1, 1))
ls, rs = [], []
for c in cs:
    left, right = motors(c, c)
    ls.append(left)
    rs.append(right)
plt.plot(cs, ls, label="predicted left motor", c="orange", alpha=0.3)
plt.plot(cs, rs, label="predicted right motor", c="blue", alpha=0.3)

##
aw = (arp + alp) / 2
rcs, lcs = [], []
for c in cs:
    if c > 0:
        rc = (aw * c - brp) / arp
        lc = (aw * c - blp) / alp
    elif c < 0:
        rc = (aw * c - brn) / arn
        lc = (aw * c - bln) / aln
    else:
        rc = 0
        lc = 0
    rcs.append(clamp(rc, -300, 300))
    lcs.append(clamp(lc, -300, 300))
responses = [motors(lc, rc) for lc, rc in zip(lcs, rcs)]
ls, rs = unzip(responses)
plt.plot(cs, rs, c="green", linestyle="dashdot", alpha=0.9, label="preprocessed right motor")
plt.plot(cs, ls, c="red", linestyle="dashed", alpha=0.3, label="preprocessed left motor")

constants = [aw, arp, brp, alp, blp, arn, brn, aln, bln]
print(", ".join([str(const) for const in constants]))

plt.legend()
plt.xlabel("command [unit]")
plt.ylabel("delta [count/second]")
plt.title("""Speed of motors over different commands (to calculate motor speed)""")
plt.grid(True)
plt.savefig(output)
plt.show()
