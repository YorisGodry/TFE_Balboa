
import collections
import smbus # type: ignore
import struct

class Regs(object):
  CTRL_REG1   = 0x20
  CTRL_REG2   = 0x21
  CTRL_REG3   = 0x22
  CTRL_REG4   = 0x23
  CTRL_REG5   = 0x24
  OUT_X_L     = 0x28

Vector = collections.namedtuple('Vector', 'x y z')

class LIS3MDL(object):
  """ Allows the RPi to fetch data from the LIS3MDL Magnetometer """
  
  def __init__(self, slave_addr = 0b0011110):
    self.bus = smbus.SMBus(1)
    self.sa = slave_addr
    self.m = Vector(0, 0, 0)
    self.setup()
      
  def setup(self):
    # 0x70 = 0b01110000
    # OM = 11 (ultra-high-performance mode for X and Y); DO = 100 (10 Hz ODR)
    self.bus.write_byte_data(self.sa, Regs.CTRL_REG1, 0x70)

    # 0x00 = 0b00000000
    # FS = 00 (+/- 4 gauss full scale)
    self.bus.write_byte_data(self.sa, Regs.CTRL_REG2, 0x00)

    # 0x00 = 0b00000000
    # MD = 00 (continuous-conversion mode)
    self.bus.write_byte_data(self.sa, Regs.CTRL_REG3, 0x00)

    # 0x0C = 0b00001100
    # OMZ = 11 (ultra-high-performance mode for Z)
    self.bus.write_byte_data(self.sa, Regs.CTRL_REG4, 0x0C)

    # 0x40 = 0b01000000
    # BDU = 1 (block data update)
    self.bus.write_byte_data(self.sa, Regs.CTRL_REG5, 0x40)
  

  def read(self):
    byte_list = self.bus.read_i2c_block_data(self.sa, Regs.OUT_X_L | 0x80, 6)
    self.m = Vector(*struct.unpack('hhh', bytes(byte_list)))
