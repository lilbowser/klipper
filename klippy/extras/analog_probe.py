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
        self.z_offset_ProbeBodyToNozzle = config.getfloat('z_offset')         # Physical offset from nozzle to bottom of laser device. Not mesurement range.
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

        self.lower_bounds_percentage = config.getfloat('bounds_percentage', 0.05, minval=0., maxval=0.49)


        # self.measurement_height = config.getfloat('measurement_height', 10, minval=0.)  # mm

        #Testing Config Vars
        self.probed_z_modifier = config.getfloat('probed_z_mod', 0.)

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
        self._last_rel_measurement_value = 0.  # RelZ
        self._last_z_result = 0.
        self._last_raw_adc_value = 0.
        
        self.z_offset_NozzleToTriggerPoint = self._get_ideal_measurement_height()
        # self.z_offset_NozzleToEndRange = self.measure_range_end - self.z_offset_ProbeBodyToNozzle

    cmd_PROBE_help = "Run probe and stop at the contact position."
    cmd_DEBUG_PROBE_help = "Displays debug info from the probe at the current toolhead position."


    # Initialization
    def handle_ready(self):
        # Load printer objects
        self.toolhead = self.printer.lookup_object('toolhead')


    def get_lift_speed(self, gcmd=None):
        return self.lift_speed
    
    def get_offsets(self):
        # self.gcode.respond_info("[get_offsets] Returning zOff: %.3f (imh: %.3f, ZOff_pb-n: %.3f)" % (self.z_offset_NozzleToTriggerPoint, self._get_ideal_measurement_height(), self.z_offset_ProbeBodyToNozzle) )
        return self.x_offset, self.y_offset, 0  #self.z_offset_NozzleToTriggerPoint


    def multi_probe_begin(self):
        pass


    def multi_probe_end(self):
        pass


    def get_status(self, eventtime):
        return {
          'last_rel_measurement_value': self._last_rel_measurement_value,
          'last_z_result': self._last_z_result,
        }


    def _take_probe_measurment(self, gcmd):
        """
        Returns self._last_z_result
        """

        fake_laser_reading = gcmd.get_float('FAKE', None)

        rel_z = self._get_rel_mesurement(gcmd) if fake_laser_reading is None else fake_laser_reading

        rel_measure_range_end = (self.measure_range_end - self.measure_range_start)
        upper_bounds_percentage = 1.0 - self.lower_bounds_percentage 
        if rel_z >= rel_measure_range_end*upper_bounds_percentage or rel_z <= rel_measure_range_end*self.lower_bounds_percentage:
            raise gcmd.error("[tpm] Analog Probe is not in mesurement range. adc=%.6f, rel_z=%.6f" % (self._last_raw_adc_value, rel_z))

        abs_dist_from_bed_to_probe_bottom = rel_z + self.measure_range_start - self.z_offset_ProbeBodyToNozzle + self.probed_z_modifier
        gcmd.respond_info("[tpm] rel_z(%.3f) + mrs(%.3f) - zOff(%.3f) + pzm(%.3f) = %.3f"
                          % (rel_z, self.measure_range_start, self.z_offset_ProbeBodyToNozzle, self.probed_z_modifier, abs_dist_from_bed_to_probe_bottom))
        
        self._last_z_result = abs_dist_from_bed_to_probe_bottom
        return self._last_z_result


    def run_probe(self, gcmd):

        # wait until toolhead is in position
        self.toolhead.wait_moves()
        gcmd.respond_info("[run_probe] Begining Analog Probe")

        # move into measurment range
        startingPos = self.toolhead.get_position()
        probing_z_pos = self._get_ideal_measurement_height()
        gcmd.respond_info("[run_probe] Moving Z from %.1f to %.1f" % (startingPos[2], probing_z_pos))
        self.toolhead.manual_move([startingPos[0],startingPos[1],probing_z_pos], self.lower_speed)
        self.toolhead.wait_moves()

        if self.dwell_time:
            self.toolhead.dwell(self.dwell_time)
        
        return startingPos[0],startingPos[1],self._take_probe_measurment(gcmd)

        
    def _get_ideal_measurement_height(self):
        half_measure_range = (self.measure_range_end + self.measure_range_start) / 2.0  # 20+30/2 -> 25
        tip_pos_z = half_measure_range - self.z_offset_ProbeBodyToNozzle  # 25 - +10 -> 15
        return tip_pos_z
        
    def _convert_adc_reading(self, adc_reading):

        adc_range_start = self.min_voltage/self.ref_voltage
        adc_range_end = self.max_voltage/self.ref_voltage
        measured_voltage = (adc_reading - adc_range_start) / (adc_range_end - adc_range_start)
        measurment_range = self.measure_range_end - self.measure_range_start
        return measured_voltage * measurment_range

    def _adc_callback(self, time, value):
        # convert to physical unit
        self._last_rel_measurement_value = self._convert_adc_reading(value)
        self._last_measurement_time = time
        # store raw adc value for debuging
        self._last_raw_adc_value = value

    def _get_rel_mesurement(self, gcmd):
        # read ADC sample
        self._adc_wait_conversion_ready(gcmd)
        return self._last_rel_measurement_value

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

        startPos = self.toolhead.get_position()

        gcmd.respond_info("[cmd_PROBE] Begining PROBE from X:%.3f Y:%.3f Z:%.3f\n"
                          % (startPos[0], startPos[1], startPos[2]))

        XYProbedZ = self.run_probe(gcmd)
        gcmd.respond_info("Result is z=%.6f" % (XYProbedZ[2],))
        self.toolhead.manual_move([startPos[0],startPos[1],startPos[2]], self.lift_speed)  # Move back to original pos
        self.toolhead.wait_moves()


    def cmd_DEBUG_PROBE(self, gcmd):
        pos = self.toolhead.get_position()

        gcmd.respond_info("Toolhead at X:%.3f Y:%.3f Z:%.3f\n"
                          % (pos[0], pos[1], pos[2]))

        probedZ = self._take_probe_measurment(gcmd)
        gcmd.respond_info("Result is abs Z=%.6f, rel_z=%.6f, adc=%.6f" % (probedZ, self._last_rel_measurement_value, self._last_raw_adc_value))
        return


def load_config(config):
    probe = AnalogProbe(config)
    config.printer.add_object('probe', probe)
    return probe