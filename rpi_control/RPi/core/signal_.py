
class Signal:
    INVALID = 1
    DELTA_INVALID = 2
    VALID = 3

    def __init__(self):
        self.value = 0
        self.delta = 0
        #self.sum = 0
        self.derivative = 0
        self.integral = 0
        self.validity = self.INVALID
    
    def extend(self, value, delta_time):
        if self.validity != self.INVALID:
            self.delta = value - self.value
            self.derivative = self.delta / delta_time
            self.validity = self.VALID
        self.value = value
        #self.sum += value
        self.integral += value * delta_time
        if self.validity == self.INVALID:
            self.validity = self.DELTA_INVALID
    
    def reset(self, reset_value=0):
        self.__init__()
        self.value = reset_value

    def pid(self, kp, ki, kd):
        return kp * self.value + ki * self.integral + kd * self.derivative

class ComplementaryFilter(Signal):
    def __init__(self):
        super().__init__()

    def update(self, rate, absolute, trust, delta_time):
        a = self.value + rate * delta_time
        b = absolute
        value = a * trust + b * (1 - trust)
        self.extend(value, delta_time)

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
    def reset(self):
        while len(self) > 0:
            self.pop()
    
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
    def reset(self):
        super().reset()
        self.sum = 0

class BoundQueueSum(QueueSum):
    def __init__(self, window_size):
        super().__init__()
        self.window_size = window_size
    def push(self, value):
        super().push(value)
        if len(self) > self.window_size:
            self.pop()
    def is_full(self):
        return len(self) == self.window_size

class MovingAverage(Signal):
    def __init__(self, window_size):
        self.window = BoundQueueSum(window_size)
        super().__init__()
    
    def extend(self, value, delta_time):
        self.window.push(value)
        super().extend(self.window.average(), delta_time)
