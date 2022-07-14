# MCP4728 dac code
#
# Copyright (C) 2018  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from . import bus
from mcp47cxb1x import PrinterSetDACCommandHelper

BACKGROUND_PRIORITY_CLOCK = 0x7fffffff00000000

class mcp4728:
    def __init__(self, config):
        self.channel_count = 4
        self.mcu = self.i2c.get_mcu()
        self.i2c = bus.MCU_I2C_from_config(config, default_addr=0x60)
        self.scale = config.getfloat('scale', 1., above=0.)
        self.last_values = [0.] * self.channel_count
        PrinterSetDACCommandHelper(config, self)
        # Configure registers
        for i, name in enumerate('abcd'):
            val = config.getfloat('channel_%s' % (name,), None,
                                  minval=0., maxval=self.scale)
            if val is not None:
                self.last_values[i] = val
                self.set_dac(i, self._scale_value(val))
            else:
                self.last_values[i] = 0
    def set_dac(self, dac, value, print_time=None):
        minclock = 0
        if print_time is not None:
            minclock = self.mcu.print_time_to_clock(print_time)

        self.i2c.i2c_write([0x40 | (dac << 1),
                            ((value >> 8) & 0x0f) | 0x80, value & 0xff], 
                            minclock=minclock,
                            reqclock=BACKGROUND_PRIORITY_CLOCK)

    def _scale_value(self, value):
        return int(value * 4095. / self.scale + .5)
    def set_dac_from_command(self, chan, value, print_time):
        self.last_values[chan] = value
        self._set_dac(chan, self._scale_value(value), print_time)
    def get_status(self, eventtime):
        return {
            'scale': self.scale,
            'values': self.last_values
            }

def load_config_prefix(config):
    return mcp4728(config)
