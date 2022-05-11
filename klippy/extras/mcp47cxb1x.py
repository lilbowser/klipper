# mcp47cxb1x DAC code
#
# Copyright (C) 2022 Sciperio
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from . import bus
import logging

BACKGROUND_PRIORITY_CLOCK = 0x7fffffff00000000

class PrinterOutputDACCommandHelper:
    def __init__(self, config, dac):
        self.printer = config.get_printer()
        self.dac = dac
        self.name = config.get_name().split()[-1]
        self.register_commands(self.name)
    def register_commands(self, name):
        # Register commands
        gcode = self.printer.lookup_object('gcode')
        gcode.register_mux_command("OUTPUT_DAC", "DAC", name,
                                   self.cmd_OUTPUT_DAC,
                                   desc=self.cmd_OUTPUT_DAC_help)
    cmd_OUTPUT_DAC_help = "Set the value of a digital-to-analog converter"
    def cmd_OUTPUT_DAC(self, gcmd):
        channel = gcmd.get_int('CHANNEL',
                               minval=1, maxval=self.dac.channel_count)
        value = gcmd.get_float('VALUE', minval=0., maxval=self.dac.scale)

        toolhead = self.printer.lookup_object('toolhead')
        toolhead.register_lookahead_callback(
            lambda print_time: self.dac.set_dac_from_command(channel-1, value,
                                                             print_time))

class mcp47cxb1x:
    def __init__(self, config):
        self.channel_count = config.getint('channel_count', 1,
                                           minval=1, maxval=2)
        self.last_values = [0] * self.channel_count
        self.i2c = bus.MCU_I2C_from_config(config, default_addr=0x60)
        self.mcu = self.i2c.get_mcu()
        self.scale = config.getfloat('scale', 1., above=0.)

        PrinterOutputDACCommandHelper(config, self)

        for i in range(self.channel_count):
            val = config.getfloat('channel_%d' % (i+1,), None,
                                minval=0., maxval=self.scale)

            if val is not None:
                self.last_values[i] = val
                self.set_dac(i, self._scale_value(val))
    def set_dac(self, dac_ch, raw_value, print_time=None):
        minclock = 0
        if print_time is not None:
            minclock = self.mcu.print_time_to_clock(print_time)
        self.i2c.i2c_write([
                            ((dac_ch << 3) & 0xF8),
                            ((raw_value >> 8) & 0x0f), raw_value & 0xff],
                            minclock=minclock,
                            reqclock=BACKGROUND_PRIORITY_CLOCK)
    def _scale_value(self, value):
        return int(value * 1023. / self.scale + .5)
    def set_dac_from_command(self, chan, value, print_time):
        self.last_values[chan] = value
        self.set_dac(chan, self._scale_value(value), print_time)
    def get_status(self, eventtime):
        return {
            'scale': self.scale,
            'values': self.last_values
            }

def load_config_prefix(config):
    return mcp47cxb1x(config)
