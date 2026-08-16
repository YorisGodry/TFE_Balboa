
# Copyright Pololu Corporation.  For more information, see https://www.pololu.com/
# *Modified

import threading
import smbus # type: ignore
import time
import struct

def subtract_16_bit(a, b):
  diff = (a - b) & 0xFFFF
  if (diff & 0x8000):
    diff -= 0x10000
  return diff

SLAVE_ADDRESS = 20

class Balboa:
    """ Allows the RPi to communicate with the balboa through the 'balboa-buffer' """
    #lock = threading.Lock()

    def __init__(self):
        self.bus = smbus.SMBus(1)
        # save the previous encoder values
        l, r = self.read_unpack(4, 4, "hh")
        self.left_encoder = l
        self.right_encoder = r

    def read_unpack(self, address, size, format):
        #Balboa.lock.acquire()
        self.bus.write_byte(SLAVE_ADDRESS, address)
        time.sleep(0.0001)
        byte_list = [self.bus.read_byte(SLAVE_ADDRESS) for _ in range(size)]
        #Balboa.lock.release()
        return struct.unpack(format, bytes(byte_list))

    def write_pack(self, address, format, *data):
        data_array = list(struct.pack(format, *data))
        #Balboa.lock.acquire()
        self.bus.write_i2c_block_data(SLAVE_ADDRESS, address, data_array)
        time.sleep(0.0001)
        #Balboa.lock.release()

    def read_encoders(self):
        l, r = self.read_unpack(4, 4, "hh")
        delta_left = subtract_16_bit(l, self.left_encoder)
        delta_right = subtract_16_bit(r, self.right_encoder)
        self.left_encoder = l
        self.right_encoder = r
        return delta_left, delta_right

    def update_motors(self, left, right):
        self.write_pack(0, "hh", left, right)
    
    SENSOR_COUNT = 5
    def read_line_sensors(self):
        return self.read_unpack(8, 2 * self.SENSOR_COUNT, "H" * self.SENSOR_COUNT)
    



    def read_uwb(self):
        return self.read_unpack(8 + 2 * self.SENSOR_COUNT, 8, "Hhhh")