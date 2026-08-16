
from balboa import Balboa
from signal_ import Signal
import constants

class Encoders:
    def __init__(self, balboa: Balboa):
        self.left = Signal()
        self.right = Signal()
        self.balboa = balboa

    def reset(self):
        self.left.reset()
        self.right.reset()
    
    def update(self, delta_time):
        """
        Read the encoders from balboa
        and update left/right signals
        [req reset]
        """
        delta_left, delta_right = self.balboa.read_encoders()
        self.left.extend(self.left.value + delta_left * constants.mm_per_count, delta_time)
        self.right.extend(self.right.value + delta_right * constants.mm_per_count, delta_time)