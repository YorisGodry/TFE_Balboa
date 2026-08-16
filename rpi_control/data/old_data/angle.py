
class Signal:
    INVALID = 1
    DELTA_INVALID = 2
    VALID = 3

    def __init__(self):
        self.value = 0
        self.delta = 0
        self.sum = 0
        self.validity = self.INVALID
    
    def new_value(self, value):
        if self.validity != self.INVALID:
            self.delta = value - self.value
            self.validity = self.VALID
        self.value = value
        self.sum += value
        if self.validity == self.INVALID:
            self.validity = self.DELTA_INVALID
    
    def reset(self, reset_value=0):
        self.__init__()
        self.value = reset_value

    def pid(self, kp, ki, kd):
        return kp * self.value + ki * self.sum + kd * self.delta

class ComplementaryFilter(Signal):
    def __init__(self):
        super().__init__()

    def update(self, rate, absolute, trust):
        a = self.value + rate
        b = absolute
        value = a * trust + b * (1 - trust)
        self.new_value(value)

class Queue:
    def __init__(self):
        self.a, self.b = [], []
    def push(self, value):
        self.a.append(value)
    def pop(self):
        if len(self.b) == 0:
            self.invert()
        return self.b.pop()
    def invert(self):
        while len(self.a) > 0:
            self.b.append(self.a.pop())
    def __len__(self):
        return len(self.a) + len(self.b)
    
class QueueSum(Queue):
    def __init__(self):
        super().__init__()
        self.sum = 0
    def push(self, value):
        self.sum += value
        super().push(value)
    def pop(self):
        value = super().pop()
        self.sum -= value
        return value
    def average(self):
        return self.sum / len(self)

class MovingAverage(Signal):
    def __init__(self, window_size):
        self.window_size = window_size
        self.window = QueueSum()
        super().__init__()
    
    def update(self, value):
        self.window.push(value)
        if len(self.window) > self.window_size:
            self.window.pop()
        self.new_value(self.window.average())










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
            
columns, length = read_csv("data/balancing_lpf.csv")
hz = 70

### SANITY CHECKS ###
lpf = MovingAverage(10)
for acc, lpf_acc in zip(columns[0][1], columns[1][1]):
    lpf.update(acc)
    assert lpf.value == lpf_acc
cf = ComplementaryFilter()
first = True
for lpf_acc, gyro_rate, control_angle in zip(columns[1][1], columns[2][1], columns[3][1]):
    if first:
        cf.reset(lpf_acc)
        first = False
    else:
        cf.update(gyro_rate, lpf_acc, 0.99)
    assert cf.value == control_angle
#####################

gyro_angle = ComplementaryFilter()
gyro_angle.reset(columns[0][1][0])
data = []
for value in columns[2][1]:
    gyro_angle.update(value, 0, 1)
    data.append(gyro_angle.value)
gyro_only = ("gyro_only", data)
columns.append(gyro_only)

old_angle = ComplementaryFilter()
old_angle.reset(columns[0][1][0])
data = []
for acc, gyro in zip(columns[0][1], columns[2][1]):
    old_angle.update(gyro, acc, 0.997)
    data.append(old_angle.value)
old = ("0.997 cf", data)
columns.append(old)

t = np.arange(0.0, (length - 0.5) * 1/70, 1/70)
for (name, data), c in zip(columns, ["red", "orange", "yellow", "green", "blue", "purple"]):
    plt.plot(t, [d/1000 for d in data], label=str(name), color=c, 
             linestyle="solid" if name in ["control_angle", "0.997 cf"] else "dashed")

plt.legend()
plt.xlabel("time")
plt.ylabel(f'angle')
plt.title('Balancing angle of balboa over time calculated by various complementary filters')
plt.grid(True)
plt.savefig("test.png")
plt.show()
