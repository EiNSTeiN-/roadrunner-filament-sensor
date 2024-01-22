import logging
import typing
import time
import math
from collections import OrderedDict
from functools import cached_property

try:
    import numpy as np
    import matplotlib.pyplot as plt
except:
    raise RuntimeError("extra dependencies are missing, did ~/roadrunner-filament-sensor/install.sh run successfully?")

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
        if type(self.lstsq[0]) != list:
            return self.lstsq[0]

    @cached_property
    def slope(self):
        if self.solution is not None:
            return self.solution[0]

    @cached_property
    def base(self):
        if self.solution is not None:
            return self.solution[1]

    @cached_property
    def mean(self):
        return np.mean(self.y)

    @cached_property
    def fitness(self):
        """ Calulate the goodness-of-fit of the least-square solution over the measured values. """
        if self.solution is None:
            return 0.0
        y1 = ((self.slope * self.x + self.base) - self.mean) ** 2
        y2 = (self.y - self.mean) ** 2
        return sum(y1) / sum(y2)

class Polynomial:
    """ Holds the result of solving a 3rd degree polynomial equation. """
    def __init__(self, coefs):
        self.coefficients = coefs

    @classmethod
    def from_values(cls, iter, deg=3):
        x = np.array(list(iter.keys()))
        y = np.array(list(iter.values()))
        coefs = np.polynomial.polynomial.polyfit(x, y, deg)
        return cls(coefs)

    def solve(self, x):
        return np.polynomial.polynomial.polyval(x, self.coefficients)

class EventsTimeline:
    def __init__(self, events):
        events = sorted(events, key=lambda e: e.eventtime)
        self._events = {e.eventtime: e for e in events}

        first_expected_motion = 0
        for i in range(len(events) - 1):
            if events[i].epos != events[i+1].epos:
                first_expected_motion = i
                break
        last_expected_motion = None
        for i in range(len(events) - 1):
            if events[-(i+1)].epos != events[-(i+2)].epos:
                last_expected_motion = -i if i != 0 else None
                break

        self._move_events = {e.eventtime: e for e in events[first_expected_motion:last_expected_motion]}
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
            return abs(self.timeline.expected_position.slope)
        else:
            return self._expected_speed

    @property
    def expected_volumetric_flow(self) -> typing.Optional[float]:
        return self.calibrator.speed_to_volumetric_flow(self.expected_speed)

    @property
    def actual_speed(self) -> typing.Optional[float]:
        if self.timeline:
            return abs(self.timeline.measured_position.slope)
        elif self.actual_distance and self.actual_duration:
            return abs(self.actual_distance) / self.actual_duration

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
    def __init__(self, calibrator, distance, speed, wanted_count, min_fitness):
        self.calibrator = calibrator
        self.distance : float = distance
        self.speed : float = speed
        self.wanted_count : int = wanted_count
        self.min_fitness : float = min_fitness
        self.datum : list[CalibrationData] = []

    @property
    def feedrate(self) -> int:
        return int(self.speed * 60)

    @property
    def volumetric_flow(self) -> float:
        return self.calibrator.speed_to_volumetric_flow(self.speed)

    def add(self):
        data = CalibrationData(self.calibrator, self.distance, self.speed)
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

    def count(self):
        return len(self.datum)
    __len__ = count

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

        self.gcode.register_mux_command(
            "FILAMENT_SENSOR_MEASURE", "SENSOR", self.sensor.name,
            self.cmd_FILAMENT_SENSOR_MEASURE,
            desc=self.cmd_FILAMENT_SENSOR_MEASURE_help)

    def _build_extrude_cmd(self, requested_distance, speed):
        cmd = "M83\n"

        dir = -1 if requested_distance < 0 else 1
        requested_distance = abs(requested_distance)

        r = 0.
        while r < requested_distance:
            remaining_distance = (requested_distance - r)
            if remaining_distance > (self.sensor.extruder.max_e_dist / 2):
                distance = self.sensor.extruder.max_e_dist / 2
            else:
                distance = remaining_distance
            cmd += "G1 E%.2f F%d\n" % (distance * dir, speed)
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

    def _ensure_stopped(self, timeout=3.):
        """ Wait until the sensor readings have a chance to settle. """
        t = self.reactor.monotonic()
        while not self.sensor.has_stopped_moving() and (self.reactor.monotonic() - t) < timeout:
            self.reactor.pause(self.reactor.monotonic() + 0.1)
        self.sensor.all_moves_ended()

    def run_script_from_command(self, gcmd, cmd):
        try:
            self.gcode.run_script_from_command(cmd)
        except Exception as e:
            gcmd.respond_raw(f"!! Error running extrude command: {e}\n")
            return False
        return True

    def _collect_calibration_data(self, gcmd, run, max_attempts=3, revert_position=False):
        index = 0
        attempt = 0

        while index < run.wanted_count:
            data = run.add()

            try:
                self.sensor.clear_move_queue()
                self.sensor.capture_history(True)

                # extrude some filament
                verb = "Extruding" if run.distance > 0 else "Retracting"
                gcmd.respond_info("%s %.2fmm (at speed=%.2fmm/s, flow=%.2fmm³/s)... %d/%d" %
                                (verb, abs(data.expected_distance), data.expected_speed, data.expected_volumetric_flow, index + 1, run.wanted_count))
                cmd = self._build_extrude_cmd(run.distance, run.feedrate)
                if not self.run_script_from_command(gcmd, cmd):
                    # an error occured running the script
                    return False
                self._ensure_stopped()
            finally:
                self.sensor.capture_history(False)

            move = self.sensor.get_combined_moves()

            if not move.measured_distance or move.duration is None:
                run.remove(data)
                gcmd.respond_raw(f"!! No movement detected\n")
                break

            # If needed, revert entire move without capturing
            if revert_position:
                cmd = self._build_extrude_cmd(-run.distance, run.feedrate)
                if not self.run_script_from_command(gcmd, cmd):
                    # an error occured running the script
                    return False
                self._ensure_stopped()

            # Check if we can find a well-fitted straight line through all data points
            data.capture_move_stats(move)
            epos = data.timeline.expected_position
            logging.info(f"expected_position slope={epos.slope} fitness={epos.fitness}")
            mpos = data.timeline.measured_position
            logging.info(f"measured_position slope={mpos.slope} fitness={mpos.fitness}")
            logging.info(f"expected_volumetric_flow={data.expected_volumetric_flow}")
            logging.info(f"slope_correction_factor={data.timeline.slope_correction_factor}")
            logging.info(f"calibration_point1=[{data.expected_volumetric_flow}, {data.timeline.slope_correction_factor}],")
            factor = data.timeline.measured_position.slope / data.timeline.expected_position.slope * 100.
            logging.info(f"calibration_point2=[{data.expected_volumetric_flow}, {factor}],")
            logging.info(f"timeline={repr(data.timeline)}")
            if mpos.fitness <= run.min_fitness:
                run.remove(data)
                attempt += 1
                gcmd.respond_raw(f"!! Failed to find well-fitted linear equation, the measured fitness was {mpos.fitness} (min. {run.min_fitness}). [{attempt}/{max_attempts}]\n")
                if attempt >= max_attempts:
                    break
            else:
                index += 1
                attempt = 0

        return True

    def _save_timeline_graph(self, timeline, fname):
        epos = timeline.expected_position
        mpos = timeline.measured_position

        fig = plt.figure(figsize=(20, 20))

        y0 = epos.y[0]
        plt.plot(epos.x, (epos.slope*epos.x + epos.base) - y0, 'r', label=f'Expected position (fitted line)', alpha=0.2)
        plt.plot(epos.x, epos.y - y0, 'o', label=f'Expected position', markersize=2)
        y0 = mpos.y[0]
        plt.plot(mpos.x, (mpos.slope*mpos.x + mpos.base) - y0, 'r', label=f'Measured position (fitted line)', alpha=0.2)
        plt.plot(mpos.x, mpos.y - y0, 'o', label=f'Measured position', markersize=2)

        plt.title(f"Position over time", loc='left')
        plt.xlabel("Time [seconds]")
        plt.ylabel("Position [mm]")
        plt.legend(loc='lower right')
        fig.savefig(fname)
        plt.close(fig)

    cmd_FILAMENT_SENSOR_MEASURE_help = \
        "Heats up the extruder, run a G1 command and report the result."
    def cmd_FILAMENT_SENSOR_MEASURE(self, gcmd):
        length = gcmd.get_int("LENGTH", 10)
        speed = gcmd.get_int("SPEED", 60 * 0.5) # 0.5mm/s
        count = gcmd.get_int("COUNT", 1)
        min_fitness = gcmd.get_float("MIN_FITNESS", 0.999)

        try:
            self._activate_extruder(gcmd)
            self._heat_to_temperature(gcmd)

            run = CalibrationRun(self, length, speed / 60, count, min_fitness)
            self._collect_calibration_data(gcmd, run)

            if len(run) == 0:
                gcmd.respond_raw("!! No data collected.\n")
                return

            for index, data in enumerate(run.datum):
                gcmd.respond_info(
                    "Run %d/%d - Requested=%.2fmm (at speed=%.2fmm/s, flow=%.2fmm³/s), measured=%.2fmm in %.2f seconds (at speed=%.2fmm/s, flow=%.2fmm³/s), %.2f%% extrusion by length, %.2f%% deviation in speed" %
                    (index + 1, run.wanted_count,
                        abs(data.expected_distance), data.expected_speed, data.expected_volumetric_flow,
                        abs(data.actual_distance or 0.0), data.actual_duration or 0.0,
                        data.actual_speed or 0.0, data.actual_volumetric_flow or 0.0,
                        data.percent_extruded or 0.0,
                        abs(data.expected_speed - (data.actual_speed or 0.)) / data.expected_speed * 100,
                    ))

            datestr = time.strftime("%Y%m%d_%H%M%S")
            for index, data in enumerate(run.datum):
                if data.timeline and gcmd.get("SAVE_GRAPH", None) == "1":
                    fname = f"FILAMENT_SENSOR_MEASURE_timeline_{index + 1}_{datestr}.png"
                    self._save_timeline_graph(data.timeline, fname)
                    gcmd.respond_info(f"Saved {fname}")
        finally:
            self._turn_off_heater(gcmd)

    cmd_CALIBRATE_FILAMENT_SENSOR_ROTATION_DISTANCE_help = \
        "Heats up the extruder and perform a retraction test " \
        "at the specified temperature, speed and length. The " \
        "command will print a recommended rotation_distance " \
        "for the filament sensor based on the measurements. " \
        "The SPEED value is in rpm, e.g. SPEED=300 is 5mm/s " \
        "(5 * 60 = 300)."
    def cmd_CALIBRATE_FILAMENT_SENSOR_ROTATION_DISTANCE(self, gcmd):
        length = gcmd.get_int("LENGTH", 25)
        speed = gcmd.get_int("SPEED", 60 * 0.5) # 0.5mm/s
        count = gcmd.get_int("COUNT", 10)
        min_fitness = gcmd.get_float("MIN_FITNESS", 0.999)

        try:
            self._activate_extruder(gcmd)
            self._heat_to_temperature(gcmd)

            run = CalibrationRun(self, -length, speed / 60, count, min_fitness)
            self._collect_calibration_data(gcmd, run, revert_position=True)

            if len(run) > 0:
                self._compute_rotation_distance_result(gcmd, run)
        finally:
            self._turn_off_heater(gcmd)

    def _deviation_result(self, run):
        distances = [abs(data.actual_distance) for data in run.datum if data.actual_distance is not None]
        if not len(distances):
            return
        deviation_range = max(distances) - min(distances)
        return ("Deviation between readings: %.3fmm (%d times min. detectable change of %.3fmm)." %
                    (deviation_range,
                    round(deviation_range / self.sensor.detectable_distance_change()),
                    self.sensor.detectable_distance_change()))

    def _repeatability_result(self, run):
        frequencies = {}
        for data in run.datum:
            if data.actual_distance is not None and data.actual_distance not in frequencies:
                frequencies[data.actual_distance] = 1
            else:
                frequencies[data.actual_distance] += 1

        sorted_frequencies = sorted(frequencies.keys(), key=lambda k: frequencies[k], reverse=True)
        if not len(sorted_frequencies):
            return

        most_frequent_distance = sorted_frequencies[0]
        rotation_distance = self.sensor.rotation_distance * abs(run.distance / most_frequent_distance)
        result = "Most frequent reading: %.2f with %d measurement(s), which implies a 'correct' rotation distance of %.7f." % \
                    (abs(most_frequent_distance), frequencies[most_frequent_distance], rotation_distance)
        return result, rotation_distance

    def _compute_rotation_distance_result(self, gcmd, run):
        stats = []
        for index, data in enumerate(run.datum):
            stats.append("Run %d/%d - requested=%.2fmm (at speed=%.2fmm/s, flow=%.2fmm³/s), measured=%.2fmm in %.2f seconds (at speed=%.2fmm/s, flow=%.2fmm³/s) = %.2f%% extruded" %
                        (index + 1, run.wanted_count,
                            abs(data.expected_distance), data.expected_speed, data.expected_volumetric_flow,
                            abs(data.actual_distance or 0.0), data.actual_duration or 0.0,
                            data.actual_speed or 0.0, data.actual_volumetric_flow or 0.0,
                            data.percent_extruded or 0.0))
        gcmd.respond_info("\n".join(stats))

        if result := self._deviation_result(run):
            gcmd.respond_info(result)

        result, rotation_distance = self._repeatability_result(run)
        if result:
            gcmd.respond_info(result)

            if f"{rotation_distance:.7f}" == f"{self.sensor.rotation_distance:.7f}":
                gcmd.respond_info("Keeping the existing rotation_distance is recommended.")

    def volumetric_flow_to_speed(self, volumetric_flow):
        return volumetric_flow / self.sensor.extruder.filament_area

    def speed_to_volumetric_flow(self, speed):
        return (self.sensor.extruder.filament_area * speed) if speed else 0.0

    cmd_CALIBRATE_MAX_FLOW_help = \
        "Heats up the extruder and perform an extrusion test " \
        "between the specified START and STOP volumetric flow rates, " \
        "incrementing by [STEP]mm³/s each time."
    def cmd_CALIBRATE_MAX_FLOW(self, gcmd):
        duration = gcmd.get_int("DURATION", 10)
        start_vflow = gcmd.get_float("START", 5.)
        stop_vflow = gcmd.get_float("STOP", 25.)
        step = gcmd.get_float("STEP", 1.)
        count = gcmd.get_int("COUNT", 5)
        min_fitness = gcmd.get_float("MIN_FITNESS", 0.999)

        try:
            self._activate_extruder(gcmd)
            self._heat_to_temperature(gcmd)

            runs = []

            vflow = start_vflow
            while vflow <= stop_vflow:
                speed = self.volumetric_flow_to_speed(vflow)
                run = CalibrationRun(self, max(1, speed * duration), max(0.01, speed), count, min_fitness)
                runs.append(run)
                ok = self._collect_calibration_data(gcmd, run)
                if not ok:
                    gcmd.respond_raw(f"!! Stopping at {vflow:.2f}mm³/s\n")
                    break
                vflow += step
            self._compute_max_flow_result(gcmd, runs, count, stop_vflow)
        finally:
            self._turn_off_heater(gcmd)

    def _group_measured_speeds(self, runs):
        points = {}
        max_vflow_without_error = None
        for run in runs:
            if len(run) == run.wanted_count:
                max_vflow_without_error = run.volumetric_flow
                points[run.speed] = [data.timeline.measured_position.slope for data in run.datum if data.timeline]
        return points, max_vflow_without_error

    def _conservative_speed_result(self, points, cutoff):
        suggested_max_speed = None
        for requested, measurements in reversed(points.items()):
            error = max([abs((requested - measured) / requested) for measured in measurements])
            if error < cutoff:
                suggested_max_speed = requested
                break

        if suggested_max_speed is None:
            return f"None of the measured extrusion moves have resulted in less than {cutoff * 100}% deviation from requested speed, " \
                   "perhaps calibrate the sensor e-steps or double-check your hardware?", None
        else:
            suggested_max_flow = self.speed_to_volumetric_flow(suggested_max_speed)
            info = f"Assuming no non-linear extrusion compensation, a conservative volumetric flow of {suggested_max_flow}mm³/s is suggested " \
                   f"which will keep E speed below {suggested_max_speed:.2f}mm/s and the deviation from requested speed within a margin of {cutoff * 100}%."
            return info, suggested_max_flow

    def _save_flow_graph(self, points, fname, conservative_max_flow, suggested_max_flow):
        fig = plt.figure(figsize=(20, 20))
        plt.plot(list(points.values()), list(points.keys()), "ro", markersize=2)
        smooth_x=np.arange(0, max(points.keys()), .1)
        plt.plot(list(points.keys()), list(points.keys()), label="Target (speed with negligible slip)", markersize=2, alpha=0.2)

        poly = Polynomial.from_values({np.average(v): k for k, v in points.items()})
        plt.plot(smooth_x, poly.solve(smooth_x), label=f"Measured speed (fitted polynomial)")

        poly_min = Polynomial.from_values({np.min(v): k for k, v in points.items()})
        poly_max = Polynomial.from_values({np.max(v): k for k, v in points.items()})
        plt.fill_between(smooth_x, poly_min.solve(smooth_x), poly_max.solve(smooth_x), alpha=0.2)

        plt.title(f"Measured vs expected extrusion speed\n" \
                  f"Compensation coefficients: {poly.coefficients!s}\n" \
                  f"Max flow without compensation: {conservative_max_flow:.2f}mm^3/s" \
                  f"Max flow with compensation: {suggested_max_flow:.2f}mm^3/s", loc='left')
        plt.xlabel("Measured speed [mm/s]")
        plt.ylabel("Expected speed [mm/s]")
        plt.legend(loc='lower right')
        fig.savefig(fname)
        plt.close(fig)

    def _compute_max_flow_result(self, gcmd, runs, stop_vflow):
        cutoff = gcmd.get_float("MAX_SPEED_DEVIATION", 0.05)

        for run in runs:
            stats = []
            for index, data in enumerate(run.datum):
                if data.timeline:
                    measured_speed = data.timeline.measured_position.slope
                    deviation = abs((run.speed - measured_speed) / run.speed)
                else:
                    deviation = 0.

                stats.append(
                    "Run %d/%d - requested=%.2fmm (at speed=%.2fmm/s, flow=%.2fmm³/s), measured=%.2fmm in %.2f seconds (at speed=%.2fmm/s, flow=%.2fmm³/s) = %.2f%% speed deviation" %
                    (index + 1, run.wanted_count,
                    data.expected_distance, data.expected_speed, data.expected_volumetric_flow,
                    data.actual_distance or 0.0, data.actual_duration or 0.0,
                    data.actual_speed or 0.0, data.actual_volumetric_flow or 0.0,
                    deviation * 100))
            gcmd.respond_info("\n".join(stats))

        points, max_vflow_without_error = self._group_measured_speeds(runs)
        logging.info(f"collected data={points}")

        if max_vflow_without_error is None:
            gcmd.respond_info(f"Not enough data to calculate maximum volumetric flow.")
            return

        if max_vflow_without_error < stop_vflow:
            gcmd.respond_info(
                f"Not all measurements were completed. Above {max_vflow_without_error}mm³/s, some measurements could not be taken. " \
                f"This likely indicates the maximum possible volumetric flow with this combination of filament, nozzle, and temperature.")

        info, conservative_max_flow = self._conservative_speed_result(points, cutoff)
        gcmd.respond_info(info)

        poly = Polynomial.from_values({np.average(v): k for k, v in points.items()})
        logging.info(f"Compensation coefficients: {poly.coefficients!s}")
        coefs = ','.join([f"{coef:0.7f}" for coef in poly.coefficients])
        gcmd.respond_info(f"Compensation coefficients: {poly.coefficients!s}.\nEnable with: ENABLE_NONLINEAR_EXTRUSION SENSOR={self.sensor.name} ENABLE=1 COEFFICIENTS={coefs}")

        max_requested_speed = self.volumetric_flow_to_speed(max_vflow_without_error)
        suggested_max_speed = np.average(points[max_requested_speed])
        suggested_max_flow = self.speed_to_volumetric_flow(suggested_max_speed)
        suggested_max_speed_after_compensation = poly.solve(suggested_max_speed)
        gcmd.respond_info(
            f"Assuming non-linear extrusion compensation is enabled, a maximum volumetric flow up to {suggested_max_flow:f}mm³/s is suggested " \
            f"which will keep E speed below {suggested_max_speed:.2f}mm/s (which corresponds to {suggested_max_speed_after_compensation:.2f}mm/s after compensation). " \
            f"If the extruder is unable to reliably sustain this speed, lowering the max volumetric flow further is recommended.")

        if gcmd.get("SAVE_GRAPH", None) == "1":
            datestr = time.strftime("%Y%m%d_%H%M%S")
            fname = f"CALIBRATE_MAX_FLOW_measured_vs_expected_speed_{datestr}.png"
            self._save_flow_graph(points, fname, conservative_max_flow, suggested_max_flow)
            gcmd.respond_info(f"Saved {fname}")
