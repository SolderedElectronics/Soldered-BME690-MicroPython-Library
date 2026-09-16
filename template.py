# FILE: template.py
# AUTHOR: [AUTHOR_NAME] @ Soldered
# BRIEF: MicroPython library for the [CHIP_NAME]
# LAST UPDATED: [YYYY-MM-DD]

from machine import I2C, Pin
from os import uname

# I2C address
DEFAULT_I2C_ADDR = 0x00

# Register addresses
# TEMPLATE_REG_EXAMPLE = 0x00


class Template:
    """
    MicroPython class for the [CHIP_NAME].
    """

    def __init__(self, i2c=None, address=DEFAULT_I2C_ADDR):
        """
        Initialize the [CHIP_NAME].

        :param i2c: Initialized I2C object (optional, auto-detected on known boards)
        :param address: I2C address of the device
        """
        if i2c is not None:
            self.i2c = i2c
        else:
            if uname().sysname in ("esp32", "esp8266", "Soldered Dasduino CONNECTPLUS"):
                self.i2c = I2C(0, scl=Pin(22), sda=Pin(21))
            else:
                raise Exception("Board not recognized, enter I2C pins manually")

        self.address = address

    def _readByte(self, reg):
        try:
            data = self.i2c.readfrom_mem(self.address, reg, 1)
            return True, data[0]
        except:
            return False, 0

    def _writeByte(self, reg, val):
        try:
            self.i2c.writeto_mem(self.address, reg, bytes([val]))
            return True
        except:
            return False
