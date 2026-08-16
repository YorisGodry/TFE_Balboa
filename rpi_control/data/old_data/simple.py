import matplotlib.pyplot as plt
import numpy as np

def read_csv(filename):
    with open(filename, "r") as file:
        columns = [(column, []) for column in file.readline().strip().split(",")]
        for line in file.readlines():
            line = line.strip().split(",")
            for (name, data), value in zip(columns, line):
                data.append(float(value))
    return columns, len(columns[0][1])
            
columns, length = read_csv("small.csv")
hz = 70

t = np.arange(0.0, length * 1/70, 1/70)
for (trust, data), c in zip(columns, ["red", "orange", "yellow", "green", "blue", "purple"]):
    if trust == "0.0" or trust == "1.0":
        plt.plot(t, data, label=str(trust), color=c, linestyle="solid")
    else:
        plt.plot(t, data, label=str(trust), color=c, linestyle="dashed")

plt.plot(t, [90 for i in range(length)])
plt.plot(t, [-90 for i in range(length)])
plt.legend()

plt.xlabel("time")
plt.ylabel(f'angle')
plt.title('Balancing angle of balboa over time calculated by various complementary filters')
plt.grid(True)
plt.savefig("test.png")
plt.show()
