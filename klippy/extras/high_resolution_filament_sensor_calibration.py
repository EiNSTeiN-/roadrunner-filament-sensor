import struct
import logging
import typing
from collections import OrderedDict
from functools import cached_property

try:
    import numpy as np
except:
    raise RuntimeError("numpy is required, did ~/roadrunner-filament-sensor/install.sh run successfully?")

class LeastSquares:
    """ Holds the result of a least-squares fitted curve. """
    def __init__(self, iter, rcond=None):
        self.x = np.array(list(iter.keys()))
        self.y = np.array(list(iter.values()))
        self.rcond = rcond
        self._lstsq = None

    @cached_property
    def lstsq(self):
        if self._lstsq is None:
            A = np.vstack([self.x, np.ones(len(self.x))]).T
            self._lstsq = np.linalg.lstsq(A, self.y, rcond=None)
        return self._lstsq

    @cached_property
    def solution(self):
        return self.lstsq[0]

    @cached_property
    def slope(self):
        return self.solution[0]

    @cached_property
    def base(self):
        return self.solution[1]

    @cached_property
    def mean(self):
        return np.mean(self.y)

    @cached_property
    def fitness(self):
        """ Calulate the goodness-of-fit of the least-square solution over the measured values. """
        y1 = ((self.slope * self.x + self.base) - self.mean) ** 2
        y2 = (self.y - self.mean) ** 2
        return sum(y1) / sum(y2)

class EventsTimeline:
    def __init__(self, events):
        self._events = {e.eventtime: e for e in sorted(events, key=lambda e: e.eventtime)}
        self._move_events = {e.eventtime: e for e in self._events.values() if e.distance > 0}
        self.expected_position = LeastSquares({e.eventtime: e.epos for e in self._move_events.values()})
        self.measured_position = LeastSquares({e.eventtime: e.position for e in self._move_events.values()})

    def __repr__(self):
        return f"{self.__class__.__name__}(events={repr(list(self._events.values()))})"

    @property
    def slope_correction_factor(self):
        """ Return a factor by which the measured slope can be
        multiplied to get the expected slope. """
        return self.expected_position.slope / self.measured_position.slope

class CalibrationData:
    def __init__(self, calibrator, expected_distance : float, expected_speed : float):
        self.calibrator = calibrator
        self.expected_distance = expected_distance
        self._expected_speed = expected_speed
        self.actual_distance = None
        self.actual_duration = None
        self.timeline = None

    def capture_move_stats(self, move):
        if move.expected_distance is not None:
            self.expected_distance = move.expected_distance
        self.actual_distance = move.measured_distance
        self.actual_duration = move.duration
        self.timeline = EventsTimeline(move.sensor_events)

    @property
    def expected_speed(self):
        if self.timeline:
            return self.timeline.expected_position.slope
        else:
            return self._expected_speed

    @property
    def expected_volumetric_flow(self) -> typing.Optional[float]:
        return self.calibrator.speed_to_volumetric_flow(self.expected_speed)

    @property
    def actual_speed(self) -> typing.Optional[float]:
        if self.timeline:
            return self.timeline.measured_position.slope
        elif self.actual_distance and self.actual_duration:
            return self.actual_distance / self.actual_duration

    @property
    def actual_volumetric_flow(self) -> typing.Optional[float]:
        return self.calibrator.speed_to_volumetric_flow(self.actual_speed)

    @property
    def percent_extruded(self) -> typing.Optional[float]:
        if self.actual_distance is not None and self.expected_distance:
            return self.actual_distance / self.expected_distance * 100.

    @property
    def deviation(self) -> typing.Optional[float]:
        if self.actual_distance is not None and self.expected_distance is not None:
            return self.actual_distance - self.expected_distance

    @property
    def percent_volumetric_flow(self) -> typing.Optional[float]:
        actual = self.actual_volumetric_flow
        expected = self.expected_volumetric_flow
        if actual is not None and expected:
            return actual / expected * 100.

    @property
    def slope_correction_factor(self) -> typing.Optional[float]:
        if self.timeline:
            return self.timeline.slope_correction_factor

    def corrected_rotation_distance(self, rotation_distance) -> typing.Optional[float]:
        if self.timeline:
            return self.slope_correction_factor * rotation_distance

class CalibrationRun:
    def __init__(self, calibrator):
        self.calibrator = calibrator
        self.datum : list[CalibrationData] = []

    def add(self, expected_distance : float, expected_speed : float):
        data = CalibrationData(self.calibrator, expected_distance, expected_speed)
        self.datum.append(data)
        return data

    def remove(self, data : CalibrationData):
        self.datum.remove(data)

    def _avg(self, prop):
        values = [getattr(d, prop) for d in self.datum]
        values = [val for val in values if val is not None]
        return sum(values) / len(values) if len(values) else 0.

    def get_data_averages(self):
        avg = CalibrationData(self.calibrator, self._avg('expected_distance'), self._avg('expected_speed'))
        avg.actual_distance = self._avg('actual_distance')
        avg.actual_duration = self._avg('actual_duration')
        return avg

    @property
    def count(self):
        return len(self.datum)

class HighResolutionFilamentSensorCalibration:
    def __init__(self, sensor):
        self.sensor = sensor
        self.printer = self.sensor.printer
        self.reactor = self.sensor.reactor
        self.gcode = self.sensor.gcode

        self.gcode.register_mux_command(
            "CALIBRATE_FILAMENT_SENSOR_ROTATION_DISTANCE", "SENSOR", self.sensor.name,
            self.cmd_CALIBRATE_FILAMENT_SENSOR_ROTATION_DISTANCE,
            desc=self.cmd_CALIBRATE_FILAMENT_SENSOR_ROTATION_DISTANCE_help)

        self.gcode.register_mux_command(
            "CALIBRATE_MAX_FLOW", "SENSOR", self.sensor.name,
            self.cmd_CALIBRATE_MAX_FLOW,
            desc=self.cmd_CALIBRATE_MAX_FLOW_help)

    def _build_extrude_cmd(self, requested_distance, speed):
        cmd = "M83\n"

        r = 0
        while r < requested_distance:
            remaining_distance = (requested_distance - r)
            if remaining_distance > (self.sensor.extruder.max_e_dist / 2):
                distance = self.sensor.extruder.max_e_dist / 2
            else:
                distance = remaining_distance
            cmd += "G1 E%.2f F%d\n" % (distance, speed)
            r += distance

        cmd += "M400\n" # wait for move to finish
        return cmd

    def _activate_extruder(self, gcmd):
        toolhead = self.printer.lookup_object('toolhead')
        if toolhead.get_extruder() is not self.sensor.extruder:
            gcmd.respond_info("Activating extruder '%s'" % (self.sensor.extruder_name, ))
            self.gcode.run_script_from_command("ACTIVATE_EXTRUDER EXTRUDER=%s" % (self.sensor.extruder_name, ))

    def _heat_to_temperature(self, gcmd):
        temp = gcmd.get_int("TEMP", 200)

        heater = self.sensor.extruder.get_heater()
        gcmd.respond_info("Heating %s to %d..." % (heater.name, temp))

        pheaters = self.printer.lookup_object('heaters')
        pheaters.set_temperature(heater, temp, True)

        gcmd.respond_info("Done heating")

    def _turn_off_heater(self, gcmd):
        # Turn off heater
        heater = self.sensor.extruder.get_heater()
        pheaters = self.printer.lookup_object('heaters')
        pheaters.set_temperature(heater, 0)

        gcmd.respond_info("Heater turned off")

    def _collect_calibration_data(self, gcmd, distance, speed, num_runs, min_fitness, max_attempts=3):
        run = CalibrationRun(self)
        index = 0
        attempt = 0

        while index < num_runs:
            data = run.add(distance, speed / 60.)

            try:
                self.sensor.clear_move_queue()
                self.sensor.capture_history(True)

                # extrude some filament
                gcmd.respond_info("Extruding %.2fmm (at speed=%.2fmm/s, flow=%.2fmm³/s)... %d/%d" %
                                (data.expected_distance, data.expected_speed, data.expected_volumetric_flow, index + 1, num_runs))
                cmd = self._build_extrude_cmd(distance, speed)
                try:
                    self.gcode.run_script_from_command(cmd)
                except Exception as e:
                    gcmd.respond_raw(f"!! Error running extrude command: {e}\n")
                    return

                # Give time for everything to settle
                t = self.reactor.monotonic()
                while not self.sensor.has_stopped_moving() and (self.reactor.monotonic() - t) < 3:
                    self.reactor.pause(self.reactor.monotonic() + 0.1)
                self.sensor.all_moves_ended()
            finally:
                self.sensor.capture_history(False)

            move = self.sensor.get_combined_moves()

            if not move.measured_distance or move.duration is None:
                run.remove(data)
                gcmd.respond_raw(f"!! No movement detected\n")
                break

            data.capture_move_stats(move)
            epos = data.timeline.expected_position
            logging.info(f"expected_position slope={epos.slope} fitness={epos.fitness}")
            mpos = data.timeline.measured_position
            logging.info(f"measured_position slope={mpos.slope} fitness={mpos.fitness}")
            logging.info(f"expected_volumetric_flow={data.expected_volumetric_flow}")
            logging.info(f"slope_correction_factor={data.timeline.slope_correction_factor}")
            logging.info(f"calibration_point=[{data.expected_volumetric_flow}, {data.timeline.slope_correction_factor}]")
            logging.info(f"timeline={repr(data.timeline)}")
            if mpos.fitness <= min_fitness:
                run.remove(data)
                attempt += 1
                gcmd.respond_raw(f"!! Failed to find well-fitted linear equation, the measured fitness was {mpos.fitness} (min. {min_fitness}). [{attempt}/{max_attempts}]\n")
                if attempt >= max_attempts:
                    break
            else:
                index += 1
                attempt = 0

        return run

    cmd_CALIBRATE_FILAMENT_SENSOR_ROTATION_DISTANCE_help = \
        "Heats up the extruder and perform an extrusion test " \
        "at the specified temperature, speed and length. The " \
        "command will print a recommended rotation_distance " \
        "for the filament sensor based on the measurements. " \
        "The SPEED value is in rpm, e.g. SPEED=300 is 5mm/s " \
        "(5 * 60 = 300)."
    def cmd_CALIBRATE_FILAMENT_SENSOR_ROTATION_DISTANCE(self, gcmd):
        try:
            self._activate_extruder(gcmd)
            self._heat_to_temperature(gcmd)

            length = gcmd.get_int("LENGTH", 25)
            speed = gcmd.get_int("SPEED", 100)
            count = gcmd.get_int("COUNT", 3)
            min_fitness = gcmd.get_float("MIN_FITNESS", 0.999)

            if run := self._collect_calibration_data(gcmd, length, speed, count, min_fitness):
                self._compute_rotation_distance_result(gcmd, run, length)
        finally:
            self._turn_off_heater(gcmd)

    def _compute_rotation_distance_result(self, gcmd, run, requested_distance):
        stats = []
        for index in range(run.count):
            data = run.datum[index]

            stats.append("Run %d/%d - requested=%.2fmm (at speed=%.2fmm/s, flow=%.2fmm³/s), measured=%.2fmm in %.2f seconds (at speed=%.2fmm/, flow=%.2fmm³/s) = %.2f%% extruded" %
                         (index + 1, run.count,
                         data.expected_distance, data.expected_speed, data.expected_volumetric_flow,
                          data.actual_distance or 0.0, data.actual_duration or 0.0,
                          data.actual_speed or 0.0, data.actual_volumetric_flow or 0.0,
                          data.percent_extruded or 0.0))
        gcmd.respond_info("\n".join(stats))

        stats = []
        avg = run.get_data_averages()
        stats.append("Avg move distance: %.2fmm" % (avg.actual_distance or 0.0, ))
        stats.append("Avg move duration: %.2f seconds" % (avg.actual_duration or 0.0, ))
        stats.append("Avg move speed: %.2fmm/s" % (avg.actual_speed or 0.0, ))
        stats.append("Avg volumetric flow: %.2fmm³/s" % (avg.actual_volumetric_flow or 0.0, ))
        deviations = list(filter(None, [d.deviation for d in run.datum]))
        if len(deviations):
            stats.append("Observed deviation range: [%.2fmm, %.2fmm]" % (min(deviations), max(deviations)))
        gcmd.respond_info("\n".join(stats))

        rotation_distances = list(filter(None, [d.corrected_rotation_distance(self.sensor.rotation_distance) for d in run.datum]))
        if len(rotation_distances):
            gcmd.respond_info("Suggested rotation_distance for %s: %.7f" % (self.sensor.name, np.mean(rotation_distances)))

            range_min = min(rotation_distances)
            range_max = max(rotation_distances)
            if self.sensor.rotation_distance > range_min and self.sensor.rotation_distance < range_max:
                gcmd.respond_info("The current rotation_distance %.3f is within the range of measured values [%.3f .. %.3f]." % \
                    (self.sensor.rotation_distance, range_min, range_max))
                gcmd.respond_info("Keeping the existing rotation_distance is recommended.")
        else:
            gcmd.respond_raw(f"!! Rotation distance could not be calculated\n")

    def volumetric_flow_to_speed(self, volumetric_flow):
        return volumetric_flow / self.sensor.extruder.filament_area

    def speed_to_volumetric_flow(self, speed):
        return (self.sensor.extruder.filament_area * speed) if speed else 0.0

    cmd_CALIBRATE_MAX_FLOW_help = \
        "Heats up the extruder and perform an extrusion test " \
        "between the specified START and STOP volumetric flow rates, " \
        "incrementing by 1mm³/s each time (unless STEP specifies " \
        "otherwise)."
    def cmd_CALIBRATE_MAX_FLOW(self, gcmd):
        try:
            self._activate_extruder(gcmd)
            self._heat_to_temperature(gcmd)

            duration = gcmd.get_int("DURATION", 5)
            start_vflow = gcmd.get_float("START", 5.)
            stop_vflow = gcmd.get_float("STOP", 25.)
            step = gcmd.get_float("STEP", 1.)
            # count = gcmd.get_int("COUNT", 3)
            min_fitness = gcmd.get_float("MIN_FITNESS", 0.999)

            run = CalibrationRun(self)
            vflow = start_vflow
            while vflow < stop_vflow:
                speed = self.volumetric_flow_to_speed(vflow)
                _run = self._collect_calibration_data(gcmd, max(1, speed * duration), max(1, speed * 60), 1, min_fitness)
                if not _run:
                    break
                if _run.count == 0:
                    gcmd.respond_raw(f"!! Stopping at {vflow:.2f}mm³/s\n")
                    break
                run.datum.append(_run.datum[0])
                vflow += step
            self._compute_max_flow_result(gcmd, run)
        finally:
            self._turn_off_heater(gcmd)

    def _compute_max_flow_result(self, gcmd, run):
        suggested_max_flow = None

        cutoff = gcmd.get_float("MIN_EXTRUSION_RATE", 98.)

        stats = []
        for index in range(run.count):
            data = run.datum[index]

            # the factor used below is the ratio between the slope of
            # the measured position over the slope of the expected position
            # during the extrusion move. the slope is the speed at which
            # the filament is moving, so this gives a measure of how much
            # under-extrusion took place.
            if data.timeline:
                factor = data.timeline.measured_position.slope / data.timeline.expected_position.slope * 100.
            else:
                factor = 0.

            if factor >= cutoff:
                suggested_max_flow = data.expected_volumetric_flow

            stats.append("Run %d/%d - requested=%.2fmm (at speed=%.2fmm/s, flow=%.2fmm³/s), measured=%.2fmm in %.2f seconds (at speed=%.2fmm/, flow=%.2fmm³/s) = %.2f%%" %
                         (index + 1, run.count,
                         data.expected_distance, data.expected_speed, data.expected_volumetric_flow,
                          data.actual_distance or 0.0, data.actual_duration or 0.0,
                          data.actual_speed or 0.0, data.actual_volumetric_flow or 0.0,
                          factor))
        gcmd.respond_info("\n".join(stats))

        if suggested_max_flow is None:
            gcmd.respond_info("None of the measured extrusion moves have resulted in a vol. flow above %.2f%% of the requested flow, " \
                              "perhaps you should calibrate the sensor e-steps or double-check your hardware?" % (cutoff, ))
        else:
            gcmd.respond_info("Suggested maximum volumetric flow for measured vol. flow above %.2f%% of expected value: %.2f" % (cutoff, suggested_max_flow, ))
