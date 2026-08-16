from balboa import Balboa
from signal_ import Signal


# extremity sensors are actually at +/-28.575 mm
SENSOR_POSITIONS = [19.05, 9.525, 0, -9.525, -19.05] 
SENSOR_COUNT = len(SENSOR_POSITIONS)

class LineSensors:
    def __init__(self, balboa: Balboa):
        self.balboa = balboa
        self.line_position = Signal()
        self.sensor_values = None

    def read_sensor_values(self):
        self.sensor_values = self.balboa.read_line_sensors()
        white_line = True
        if white_line:
            self.sensor_values = tuple([2500 - value for value in self.sensor_values])
    
    def update(self, delta_time):
        self.read_sensor_values()
        min_ = min(self.sensor_values)
        max_ = max(self.sensor_values)
        if min_ == max_:
            self.line_position.extend(0, delta_time)
        else:
            weighted_sum = 0
            calibrated_values = [(value - min_) / (max_ - min_) for value in self.sensor_values]
            for i, position in zip(range(SENSOR_COUNT), SENSOR_POSITIONS):
                weighted_sum += position * calibrated_values[i]
            position = weighted_sum / sum(calibrated_values)
            self.line_position.extend(position, delta_time)

    def reset(self):
        self.line_position.reset()
        