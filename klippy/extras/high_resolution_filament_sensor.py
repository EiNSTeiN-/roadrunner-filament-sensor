# High Resolution Filament Sensor
#
# Copyright (C) 2023 Francois Chagnon <fc@francoischagnon.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import struct
import logging
from . import bus, filament_switch_sensor

DEFAULT_TARGET_ADDR = 0x40
DEFAULT_SPEED = 100000

CHECK_RUNOUT_TIMEOUT = .250 # read sensor value every 250ms
HISTORY_SECONDS = 2.
HISTORY_NUM_EVENTS = int((HISTORY_SECONDS / CHECK_RUNOUT_TIMEOUT) + 1) # history into the past to calculate stats

class MagnetState:
    NOT_DETECTED = 1
    TOO_WEAK = 2
    TOO_STRONG = 3
    DETECTED = 4

    VALUES = {
        NOT_DETECTED: "not detected",
        TOO_WEAK: "too weak",
        TOO_STRONG: "too strong",
        DETECTED: "detected",
    }

    def __init__(self, value):
        self.value = value

    def __str__(self):
        return MagnetState.VALUES.get(self.value, "unknown")

    def __repr__(self):
        return "%s(value=%s)" % (self.__class__.__name__, repr(self.value))

class SensorRegister:
    MAGNET_STATE = 0x00
    FILAMENT_PRESENCE = 0x01
    ANGLE_CHANGE = 0x02

class RunoutHelper(filament_switch_sensor.RunoutHelper):
    def cmd_QUERY_FILAMENT_SENSOR(self, gcmd):
        msg = "Filament Sensor %s: filament %s, motion %s\n%s" % (self.name,
            "detected" if self.filament_present else "not detected",
            "", self.get_status(None))
        gcmd.respond_info(msg)

class SensorEvent:
    def __init__(self, eventtime, angle_change, distance, epos):
        self.eventtime = eventtime
        self.angle_change = angle_change
        self.distance = distance
        self.epos = epos

    def __repr__(self):
        return "SensorEvent(eventtime=%s, angle_change=%s, distance=%s, epos=%s)" % (
            self.eventtime, self.angle_change, self.distance, self.epos)

class HighResolutionFilamentSensor:
    def __init__(self, config):
        self.name = config.get_name().split()[-1]
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.reactor = self.printer.get_reactor()
        self.runout_helper = RunoutHelper(config)
        self.runout_helper.get_status = self.get_status
        self.estimated_print_time = None

        self.i2c = bus.MCU_I2C_from_config(config, DEFAULT_TARGET_ADDR, DEFAULT_SPEED)
        self.mcu = self.i2c.mcu

        self.extruder_name = config.get('extruder')
        self.invert_direction = config.getboolean('invert_direction', False)
        self.rotation_distance = config.getfloat('rotation_distance', minval=0)
        self.underextrusion_max_rate = config.getfloat('underextrusion_max_rate', minval=0.0, maxval=1.0)
        self.underextrusion_period = config.getfloat('underextrusion_period', minval=0.0)
        self.reset_position()

        self._history = []
        self._is_calibrating = False
        self._is_printing = False
        self._unhealthy_condition = False
        self._runout_condition = False
        self._underextrusion_start_time = None
        self._underextrusion_detected = False

        self._magnet_state = MagnetState(0xff)
        self._sensor_connected = False
        self._filament_present = False

        self._sensor_update_timer = self.reactor.register_timer(
                self._sensor_update_event)
        self.printer.register_event_handler('klippy:ready',
                self._handle_ready)
        self.printer.register_event_handler('idle_timeout:printing',
                self._handle_printing)
        self.printer.register_event_handler('idle_timeout:ready',
                self._handle_not_printing)
        self.printer.register_event_handler('idle_timeout:idle',
                self._handle_not_printing)

        self.gcode.register_mux_command(
            "CALIBRATE_FILAMENT_SENSOR_ROTATION_DISTANCE", "SENSOR", self.name,
            self.cmd_CALIBRATE_FILAMENT_SENSOR_ROTATION_DISTANCE,
            desc=self.cmd_CALIBRATE_FILAMENT_SENSOR_ROTATION_DISTANCE_help)

        self.gcode.register_mux_command(
            "CALIBRATE_MAX_FLOW", "SENSOR", self.name,
            self.cmd_CALIBRATE_MAX_FLOW,
            desc=self.cmd_CALIBRATE_MAX_FLOW_help)
    
    def reset_position(self):
        self.position = 0.0

    def get_history_period(self):
        """ Length of time elapsed between the first and last events in the history buffer. """
        if len(self._history) > 0:
            return self._history[0].eventtime - self._history[-1].eventtime
        else:
            return None
        
    def get_motion_direction(self, distance):
        if distance == 0.0:
            return "idle"
        elif distance > 0:
            return "extruding"
        else:
            return "reversing"
    
    def _measured_underextrusion_rate(self):
        """ Return a measure of how much we are actually extruding 
        compared to how much was expected over the events kept in 
        the history buffer. The rate is returned as a value from 
        0.0 to 1.0, 0 means the measured rate matches the expected 
        rate and 1 meaning no extrusion at all when some was expected. """

        expected_distance = self._history[0].epos - self._history[-1].epos
        actual_distance = sum([evt.distance for evt in self._history])

        if expected_distance < 0.1:
            # TODO: get a measure of the sensor's expected resolution
            return 0.0
        else:
            return max(0.0, 1. - (actual_distance / expected_distance))

    def get_status(self, eventtime):
        distance = sum([evt.distance for evt in self._history])
        period = self.get_history_period()

        if len(self._history) > 1:
            expected_distance = self._history[0].epos - self._history[-1].epos
        
        if period is not None and period > 0:
            speed = abs(distance) / period
        else:
            speed = None

        return {
            "enabled": bool(self.runout_helper.sensor_enabled),
            "sensor_connected": self._sensor_connected,
            "last_eventtime": self._history[0].eventtime,
            "magnet_state": str(self._magnet_state),
            "filament_detected": bool(self._filament_present),
            "motion": {
                "detected": bool(distance != 0),
                "direction": self.get_motion_direction(distance),
                "expected_distance": expected_distance,
                "measured_distance": distance,
                "measured_speed": speed,
                "measured_volumetric_flow": self._speed_to_volumetric_flow(speed) if speed else 0.0,
            },
            "underextrusion_rate": self._measured_underextrusion_rate() if self._is_printing else 0.0,
            "underextrusion_detected": self._underextrusion_detected,
            "runout": self._runout_condition,
            "position": self.position,
        }

    def read_reg1(self, reg):
        params = self.i2c.i2c_read([reg], 1)
        return bytearray(params['response'])[0]

    def read_reg4(self, reg):
        params = self.i2c.i2c_read([reg], 4)
        return bytearray(params['response'])

    def _get_extruder_pos(self, eventtime=None):
        if self.estimated_print_time is None:
            return None

        if eventtime is None:
            eventtime = self.reactor.monotonic()

        print_time = self.estimated_print_time(eventtime)
        return self.extruder.find_past_position(print_time)

    def _update_state_from_sensor(self):
        eventtime = self.reactor.monotonic()

        self._magnet_state = MagnetState(self.read_reg1(SensorRegister.MAGNET_STATE))
        self._sensor_connected = self._magnet_state.value != 0xff
        self._filament_present = self.read_reg1(SensorRegister.FILAMENT_PRESENCE) == 1

        angle_change, = struct.unpack('<l', self.read_reg4(SensorRegister.ANGLE_CHANGE))

        if self.invert_direction:
            angle_change = angle_change * -1
        angle_change = angle_change / 4096. * 360.

        distance = self.rotation_distance * angle_change / 360.
        self.position += distance

        event = SensorEvent(eventtime, angle_change, distance, self._get_extruder_pos(eventtime))
        self._history.insert(0, event)

        if self._is_calibrating is False:
            while len(self._history) > HISTORY_NUM_EVENTS:
                self._history.pop()

    def _handle_ready(self):
        logging.info("[%s] ready" % (self.__class__.__name__, ))

        self.extruder = self.printer.lookup_object(self.extruder_name)
        self.estimated_print_time = self.printer.lookup_object('mcu').estimated_print_time
        self.reactor.update_timer(self._sensor_update_timer, self.reactor.NOW)

    def _handle_printing(self, print_time):
        logging.info("[%s] printing" % (self.__class__.__name__, ))
        self._runout_condition = False
        self._underextrusion_start_time = None
        self._underextrusion_detected = False
        self._is_printing = True

    def _handle_not_printing(self, print_time):
        logging.info("[%s] not printing" % (self.__class__.__name__, ))
        self._is_printing = False
    
    def _respond_error(self, msg):
        logging.warning(msg)
        lines = msg.strip().split('\n')
        if len(lines) > 1:
            self.gcode.respond_info("\n".join(lines), log=False)
        self.gcode.respond_raw('!! %s' % (lines[0].strip(),))

    def _exec_gcode(self, prefix, template):
        try:
            self.gcode.run_script(prefix + template.render() + "\nM400")
        except Exception:
            logging.exception("Script running error")
        self.runout_helper.min_event_systime = self.reactor.monotonic() + self.runout_helper.event_delay

    def _runout_event_handler(self, eventtime):
        # Pausing from inside an event requires that the pause portion
        # of pause_resume execute immediately.
        pause_prefix = ""
        if self.runout_helper.runout_pause:
            pause_resume = self.printer.lookup_object('pause_resume')
            pause_resume.send_pause_command()
            pause_prefix = "PAUSE\n"
            self.printer.get_reactor().pause(eventtime + self.runout_helper.pause_delay)
        self._exec_gcode(pause_prefix, self.runout_helper.runout_gcode)

    def _is_sensor_healthy(self):
        """ The sensor can stop responding if micropython crashes, 
        or the magnet can be too far from the hall rotary encoder for a reading."""
        return self._sensor_connected and self._magnet_state.value == MagnetState.DETECTED

    def _sensor_unhealthy_reason(self):
        if not self._sensor_connected:
            return "no data from sensor"
        if self._magnet_state.value != MagnetState.DETECTED:
            return "magnet %s" % (str(self._magnet_state), )
        return "unknown reason"

    def _is_runout_condition(self):
        if not self._filament_present:
            self._respond_error("Detected filament not present")
            return True

        rate = self._measured_underextrusion_rate()
        if rate > self.underextrusion_max_rate:
            if self._underextrusion_start_time is None:
                self._underextrusion_start_time = self.reactor.monotonic()
                self._respond_error("Detected %.2f%% underextrusion starting at %.2f" % 
                                    (rate * 100, self._underextrusion_start_time))
                return False
            elif self._underextrusion_start_time + self.underextrusion_period < self.reactor.monotonic():
                if not self._underextrusion_detected:
                    self._respond_error("Detected sustained underextrusion for over %.2fs" % 
                                        (self.underextrusion_period, ))
                    self._underextrusion_detected = True
                return True
        elif self._underextrusion_start_time is not None:
            self.gcode.respond_info("Underextrusion cleared after %.2fs" % 
                                    (self.reactor.monotonic() - self._underextrusion_start_time), True)
            self._underextrusion_start_time = None
            self._underextrusion_detected = False
        
        return False

    def _check_extrusion_issues(self, eventtime):
        if not self._is_printing or self._is_calibrating:
           return
        
        # Sensor has become unhealthy, we run the runout code
        if not self._is_sensor_healthy():
            if not self._unhealthy_condition:
                self._respond_error("Sensor %s has become unhealthy (%s), runout triggered..." % 
                                    (self._sensor_unhealthy_reason(), ))
                self._runout_event_handler(eventtime)
                self._unhealthy_condition = True
            return
        elif self._unhealthy_condition:
            self.gcode.respond_info("Sensor %s has become healthy again")
            self._unhealthy_condition = False

        if not self._runout_condition and self._is_runout_condition():
            self._runout_event_handler(eventtime)
            self._runout_condition = True
        
    def _sensor_update_event(self, eventtime):
        self._update_state_from_sensor()

        if eventtime >= self.runout_helper.min_event_systime and self.runout_helper.sensor_enabled:
            self._check_extrusion_issues(eventtime)

        return eventtime + CHECK_RUNOUT_TIMEOUT

    def _build_extrude_cmd(self, requested_distance, speed):
        cmd = "M83\n"

        r = 0
        while r < requested_distance:
            if self.extruder.max_e_dist < requested_distance:
                distance = self.extruder.max_e_dist
            else:
                distance = requested_distance
            cmd += "G1 E%.2f F%d\n" % (distance, speed)
            r += distance

        cmd += "M400" # wait for move to finish
        return cmd

    def _activate_extruder(self, gcmd):
        toolhead = self.printer.lookup_object('toolhead')
        if toolhead.get_extruder() is not self.extruder:
            gcmd.respond_info("Activating extruder '%s'" % (self.extruder.name, ))
            self.gcode.run_script_from_command("ACTIVATE_EXTRUDER EXTRUDER=%s" % (self.extruder.name, ))

    def _heat_to_temperature(self, gcmd):
        temp = gcmd.get_int("TEMP", 200)

        heater = self.extruder.get_heater()
        gcmd.respond_info("Heating %s to %d..." % (heater.name, temp))

        pheaters = self.printer.lookup_object('heaters')
        pheaters.set_temperature(heater, temp, True)

        gcmd.respond_info("Done heating")

    def _turn_off_heater(self, gcmd):
        # Turn off heater
        heater = self.extruder.get_heater()
        pheaters = self.printer.lookup_object('heaters')
        pheaters.set_temperature(heater, 0)

        gcmd.respond_info("Heater turned off")

    def _collect_calibration_data(self, gcmd, distance, speed, num_runs):
        self._is_calibrating = True

        runs = []
        for index in range(num_runs):
            data = {}
            runs.append(data)

            self._history = []
            data['expected_speed'] = speed / 60.
            data['initial_sensor_pos'] = self.position
            data['initial_extruder_pos'] = self._get_extruder_pos()

            # extrude some filament
            gcmd.respond_info("Extruding %.2fmm (at speed=%.2fmm/s, flow=%.2fmm³/s)... %d/%d" % 
                              (distance, speed / 60., self._speed_to_volumetric_flow(speed / 60.), index + 1, num_runs))
            cmd = self._build_extrude_cmd(distance, speed)
            self.gcode.run_script_from_command(cmd)

            # Give time for everything to settle
            self.reactor.pause(self.reactor.monotonic() + 1.)

            data['final_sensor_pos'] = self.position
            data['final_extruder_pos'] = self._get_extruder_pos()
            data['expected_distance'] = data['final_extruder_pos'] - data['initial_extruder_pos']
            data['actual_distance'] = data['final_sensor_pos'] - data['initial_sensor_pos']
            data['history'] = self._history[:]

            move_history = [evt for evt in self._history if evt.distance != 0.]
            move_times = [evt.eventtime for evt in move_history]
            started, ended = min(move_times), max(move_times)
            data['actual_duration'] = ended - started
        
        self._is_calibrating = False
        return runs

    cmd_CALIBRATE_FILAMENT_SENSOR_ROTATION_DISTANCE_help = \
        "Heats up the extruder and perform an extrusion test " \
        "at the specified temperature, speed and length. The " \
        "command will print a recommended rotation_distance " \
        "for the filament sensor based on the measurements. " \
        "The SPEED value is in rpm, e.g. SPEED=300 is 5mm/s " \
        "(5 * 60 = 300)."
    def cmd_CALIBRATE_FILAMENT_SENSOR_ROTATION_DISTANCE(self, gcmd):
        self._activate_extruder(gcmd)
        self._heat_to_temperature(gcmd)

        length = gcmd.get_int("LENGTH", 25)
        speed = gcmd.get_int("SPEED", 100)
        count = gcmd.get_int("COUNT", 3)

        runs = self._collect_calibration_data(gcmd, length, speed, count)
        self._compute_rotation_distance_result(gcmd, runs)

        self._turn_off_heater(gcmd)

    def _compute_rotation_distance_result(self, gcmd, runs):
        distances = []
        durations = []
        speeds = []
        deviations = []
        volumetric_flows = []
        
        stats = []
        for index in range(len(runs)):
            data = runs[index]

            move_duration = data['actual_duration']
            durations.append(move_duration)

            expected_distance = data['expected_distance']
            actual_distance = data['actual_distance']
            distances.append(actual_distance)
            percent_extruded = actual_distance / float(expected_distance) * 100.
            deviations.append(actual_distance - expected_distance)

            move_speed = actual_distance / move_duration
            speeds.append(move_speed)

            vflow = self._speed_to_volumetric_flow(move_speed)
            volumetric_flows.append(vflow)
            
            stats.append("Run %d/%d - requested=%.2fmm (at speed=%.2fmm/s, flow=%.2fmm³/s), measured=%.2fmm in %.2f seconds (at speed=%.2fmm/, flow=%.2fmm³/s) = %.2f%% extruded" % 
                         (index + 1, len(runs), expected_distance, data['expected_speed'], self._speed_to_volumetric_flow(data['expected_speed']),
                          actual_distance, move_duration, move_speed, vflow, percent_extruded))
        gcmd.respond_info("\n".join(stats))

        stats = []
        avg_distance = sum(distances) / len(distances)
        stats.append("Avg move distance: %.2fmm" % (avg_distance, ))
        stats.append("Avg move duration: %.2f seconds" % (sum(durations) / len(durations), ))
        stats.append("Avg move speed: %.2fmm/s" % (sum(speeds) / len(speeds), ))
        stats.append("Avg volumetric flow: %.2fmm³/s" % (sum(volumetric_flows) / len(volumetric_flows), ))
        stats.append("Observed deviation range: [%.2fmm, %.2fmm]" % (min(deviations), max(deviations)))
        gcmd.respond_info("\n".join(stats))

        rotation_distance = self.rotation_distance * float(expected_distance) / avg_distance
        gcmd.respond_info("Suggested rotation_distance for %s: %.7f" % (self.name, rotation_distance))

    def _volumetric_flow_to_speed(self, volumetric_flow):
        return volumetric_flow / self.extruder.filament_area

    def _speed_to_volumetric_flow(self, speed):
        return self.extruder.filament_area * speed

    def _avg(self, array, key):
        values = [d[key] for d in array]
        return sum(values) / len(values)

    cmd_CALIBRATE_MAX_FLOW_help = \
        "Heats up the extruder and perform an extrusion test " \
        "between the specified START and STOP volumetric flow rates, " \
        "incrementing by 1mm³/s each time (unless STEP specifies " \
        "otherwise)."
    def cmd_CALIBRATE_MAX_FLOW(self, gcmd):
        self._activate_extruder(gcmd)
        self._heat_to_temperature(gcmd)

        duration = gcmd.get_int("DURATION", 5)
        start_vflow = gcmd.get_float("START", 5.)
        stop_vflow = gcmd.get_float("STOP", 25.)
        step = gcmd.get_float("STEP", 1.)
        count = gcmd.get_int("COUNT", 3)

        runs_averages = []
        vflow = start_vflow
        while vflow < stop_vflow:
            speed = self._volumetric_flow_to_speed(vflow)
            runs = self._collect_calibration_data(gcmd, max(1, speed * duration), max(1, speed * 60), count)
            averages = {}
            averages['expected_speed'] = self._avg(runs, 'expected_speed')
            averages['expected_distance'] = self._avg(runs, 'expected_distance')
            averages['actual_distance'] = self._avg(runs, 'actual_distance')
            averages['actual_duration'] = self._avg(runs, 'actual_duration')
            runs_averages.append(averages)
            vflow += step
        self._compute_max_flow_result(gcmd, runs_averages)

        self._turn_off_heater(gcmd)

    def _compute_max_flow_result(self, gcmd, runs):
        suggested_max_flow = None

        cutoff = gcmd.get_float("MIN_EXTRUSION_RATE", 99.)
        
        stats = []
        for index in range(len(runs)):
            data = runs[index]

            expected_speed = data['expected_speed']
            expected_distance = data['expected_distance']
            actual_distance = data['actual_distance']
            move_duration = data['actual_duration']

            percent_extruded = actual_distance / float(expected_distance) * 100.
            move_speed = actual_distance / move_duration

            if percent_extruded >= cutoff:
                suggested_max_flow = self._speed_to_volumetric_flow(expected_speed)

            stats.append("Run %d/%d - requested=%.2fmm (at speed=%.2fmm/s, flow=%.2fmm³/s), measured=%.2fmm in %.2f seconds (at speed=%.2fmm/, flow=%.2fmm³/s) = %.2f%% extruded" % 
                         (index + 1, len(runs), expected_distance, expected_speed, self._speed_to_volumetric_flow(expected_speed),
                          actual_distance, move_duration, move_speed, self._speed_to_volumetric_flow(move_speed), percent_extruded))
        gcmd.respond_info("\n".join(stats))

        if suggested_max_flow is None:
            gcmd.respond_info("None of the measured extrusion moves have resulted in a length above %.2f%% of the requested move, " \
                              "perhaps you should calibrate the sensor e-steps or double-check your hardware?" % (cutoff, ))
        else:
            gcmd.respond_info("Suggested maximum volumetric flow for extrusion rate above %.2f%%: %.2f" % (cutoff, suggested_max_flow, ))

def load_config_prefix(config):
    return HighResolutionFilamentSensor(config)
