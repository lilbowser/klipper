# Analog Distance Probe Support
#
# Copyright (C) 2022 Sciperio
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging
import pins
from . import probe


SAMPLE_TIME = 0.001
SAMPLE_COUNT = 8
REPORT_TIME = 0.300
RANGE_CHECK_COUNT = 4

class AnalogProbe:

    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()

        # printer objects
        self.gcode = self.toolhead = self.mcu_adc = None
        self.printer.register_event_handler("klippy:ready", self.handle_ready)

        #Load Config
        self.x_offset = config.getfloat('x_offset', 0.)
        self.y_offset = config.getfloat('y_offset', 0.)
        self.z_offset = config.getfloat('z_offset')         # Physical offset from nozzle to bottom of laser device. Not mesurement range.
                                                            # + is above nozzle, - is below nozzle.
        # logging.info("Got offsets")
        self.lower_speed = config.getfloat('lower_speed', above=0.)
        self.lift_speed = config.getfloat('lift_speed', above=0.)

        self.dwell_time = config.getfloat('dwell_time', 0.5, minval=0.)
        # self.trigger_point = config.getfloat('trigger_point', 0.5, minval=0.)
        
        self.min_voltage = config.getfloat('min_voltage', 0., minval=0.)
        self.max_voltage = config.getfloat('max_voltage', 3.3, minval=0.)
        self.ref_voltage = config.getfloat('ref_voltage', 3.3, minval=0.)


        self.measure_range_start = config.getfloat('measure_range_start', 20., minval=0.)  # mm
        self.measure_range_end = config.getfloat('measure_range_end', 30., above=self.measure_range_start)  # mm

        # self.measurement_height = config.getfloat('measurement_height', 10, minval=0.)  # mm

        #Start ADC
        ppins = self.printer.lookup_object('pins')
        self.mcu_adc = ppins.setup_pin('adc', config.get('sensor_pin'))

        self.mcu_adc.setup_minmax(SAMPLE_TIME, SAMPLE_COUNT)
                                #   minval=0., maxval=5.,
                                #   range_check_count=RANGE_CHECK_COUNT)
        self.mcu_adc.setup_adc_callback(REPORT_TIME, self._adc_callback)

        # register gcode commands
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_command('PROBE', self.cmd_PROBE,
                                    desc=self.cmd_PROBE_help)
        # self.gcode.register_command('QUERY_PROBE', self.cmd_PROBE, desc=self.cmd_PROBE_help)
        
        self.gcode.register_command('DEBUG_PROBE', self.cmd_DEBUG_PROBE,
                                    desc=self.cmd_DEBUG_PROBE_help)
        self._last_measurement_time = 0.
        self._last_measurement_value = 0.
        self._last_z_result = 0.
        self._last_raw_adc_value = 0.

    cmd_PROBE_help = "Run probe and stop at the contact position."
    cmd_DEBUG_PROBE_help = "Displays debug info from the probe at the current toolhead position."


    # Initialization
    def handle_ready(self):
        # Load printer objects
        self.toolhead = self.printer.lookup_object('toolhead')

    def get_lift_speed(self, gcmd=None):
        return self.lift_speed
    

    def get_offsets(self):
        return self.x_offset, self.y_offset, self.z_offset


    def multi_probe_begin(self):
        pass


    def multi_probe_end(self):
        pass


    def get_status(self, eventtime):
        return {
          'last_measurement_value': self._last_measurement_value,
          'last_z_result': self._last_z_result,
        }


    def run_probe(self, gcmd):

        # wait until toolhead is in position
        self.toolhead.wait_moves()

        # move into measurment range
        pos = self.toolhead.get_position()
        self.toolhead.manual_move([pos[0],pos[1],self._get_ideal_measurement_height()], self.lower_speed)
        self.toolhead.wait_moves()

        if self.dwell_time:
            self.toolhead.dwell(self.dwell_time)
        
        rel_z = self._get_rel_mesurement(gcmd)

        rel_measure_range_end = (self.measure_range_end - self.measure_range_start)
        if rel_z >= rel_measure_range_end*0.95 or rel_z <= rel_measure_range_end*0.05:
            raise gcmd.error("Analog Probe is not in mesurement range. adc=%.6f, rel_z=%.6f" % (self._last_raw_adc_value, rel_z))

        abs_dist_from_bed_to_nozzle = rel_z + self.measure_range_start + self.z_offset
        gcmd.respond_info("Result is z=%.6f" % (abs_dist_from_bed_to_nozzle,))
        self._last_z_result = abs_dist_from_bed_to_nozzle
        return pos[0], pos[1], abs_dist_from_bed_to_nozzle

    def _get_ideal_measurement_height(self):
        half_measure_range = (self.measure_range_end + self.measure_range_start) / 2.0  # 20+30/2 -> 25
        tip_pos_z = half_measure_range - self.z_offset  # 25 - +10 -> 15
        return tip_pos_z
        
    def _convert_adc_reading(self, adc_reading):

        adc_range_start = self.max_voltage/self.ref_voltage
        adc_range_end = self.min_voltage/self.ref_voltage
        measured_voltage = (adc_reading - adc_range_start) / (adc_range_end - adc_range_start)
        measurment_range = self.measure_range_end - self.measure_range_start
        return measured_voltage * measurment_range

    def _adc_callback(self, time, value):
        # convert to physical unit
        self._last_measurement_value = self._convert_adc_reading(value)
        self._last_measurement_time = time
        # store raw adc value for debuging
        self._last_raw_adc_value = value

    def _get_rel_mesurement(self, gcmd):
        # read ADC sample
        self._adc_wait_conversion_ready(gcmd)
        return self._last_measurement_value

    def _adc_wait_conversion_ready(self, gcmd):
        last_time = self._last_measurement_time
        clocksync = self.mcu_adc.get_mcu()._clocksync
        clock = clocksync.print_time_to_clock(last_time)
        last_sys_time = clocksync.estimate_clock_systime(clock)
        for n in range(1,10):
          # wait shortly after the timer has called _sample_timer
          self.reactor.pause(last_sys_time + n*REPORT_TIME + 0.0001)
          if self._last_measurement_time != last_time:
            return

        # callback not called after 10 report time intervals -> error
        raise gcmd.error("Timeout waiting for ADC value.")

    def cmd_PROBE(self, gcmd):
        pos = self.toolhead.get_position()

        gcmd.respond_info("PROBE at X:%.3f Y:%.3f Z:%.3f\n"
                          % (pos[0], pos[1], pos[2]))

        pos = self.run_probe(gcmd)
        self.toolhead.manual_move([pos[0],pos[1],pos[2]], self.lower_speed)
        self.toolhead.wait_moves()


    def cmd_DEBUG_PROBE(self, gcmd):
        pos = self.toolhead.get_position()

        gcmd.respond_info("Toolhead at X:%.3f Y:%.3f Z:%.3f\n"
                          % (pos[0], pos[1], pos[2]))

        # Get relative probe mesurement
        rel_z = self._get_rel_mesurement(gcmd)
        # Get raw probe reading
        raw_adc_value = self._last_raw_adc_value

        rel_measure_range_end = (self.measure_range_end - self.measure_range_start)
        if rel_z >= rel_measure_range_end*0.95 or rel_z <= rel_measure_range_end*0.05:
            raise gcmd.error("Analog Probe is not in mesurement range. adc=%.6f, rel_z=%.6f" % (raw_adc_value, rel_z))

        abs_dist_from_bed_to_nozzle = rel_z + self.measure_range_start + self.z_offset
        gcmd.respond_info("Result is abs Z=%.6f, rel_z=%.6f, adc=%.6f" % (abs_dist_from_bed_to_nozzle, rel_z, raw_adc_value))
        self._last_z_result = abs_dist_from_bed_to_nozzle



def load_config(config):
    probe = AnalogProbe(config)
    config.printer.add_object('probe', probe)
    return probe