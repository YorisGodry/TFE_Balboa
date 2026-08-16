
import matplotlib.pyplot as plt
import numpy as np

def read_csv(filename):
    with open(filename, "r") as file:
        columns = [(column, []) for column in file.readline().strip().split(",")]
        for line in file.readlines():
            line = line.strip().split(",")
            for (name, data), value in zip(columns, line):
                try:
                    data.append(int(value))
                except:
                    data.append(value == "True")
    return columns, len(columns[0][1])
            
columns, length = read_csv("data/motors_linear.csv")
hz = 2



def scale(data):
    return [value * 0.248 * 2 for value in data]
lm, rm, le, re, increasing = tuple([columns[i][1] for i in range(5)])
le, re = scale(le), scale(re)
lmu, rmu, leu, reu, lmd, rmd, led, red = tuple([[] for _ in range(8)])
for m, e, up in zip(lm, le, increasing):
    if up:
        lmu.append(m)
        leu.append(e)
    else:
        lmd.append(m)
        led.append(e)
for m, e, up in zip(rm, re, increasing):
    if up:
        rmu.append(m)
        reu.append(e)
    else:
        rmd.append(m)
        red.append(e)

plt.scatter(lmu, leu, s=25, alpha=0.6, c="blue", label="left up")
plt.scatter(rmu, reu, s=25, alpha=0.8, c="orange", label="right up")

plt.scatter(lmd, led, s=25, alpha=0.6, c="black", label="left down")
plt.scatter(rmd, red, s=25, alpha=0.8, c="red", label="right down")

plt.legend()
plt.xlabel("motor control")
plt.ylabel("motor speed (mm/sec)")
plt.title('Motor speed of the robot on different speed settings')
plt.grid(True)
plt.savefig("motors.png")
plt.show()
