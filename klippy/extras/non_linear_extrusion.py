import logging
import ast
import numpy
import math

class NonLinearExtrusion:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode_move = None
        self.toolhead = None

        self.extruder_name = config.get('extruder')
        self.extruder = None

        self.coefficients = None
        self.enabled = False
        self._next_transform = None
        self._logging = config.getboolean('debug', False)

        self.printer.register_event_handler('klippy:ready', self._handle_ready)

        self.gcode.register_mux_command(
            "ENABLE_NONLINEAR_EXTRUSION", "EXTRUDER", self.extruder_name,
            self.cmd_ENABLE_NONLINEAR_EXTRUSION,
            desc=self.cmd_ENABLE_NONLINEAR_EXTRUSION_help)

    cmd_ENABLE_NONLINEAR_EXTRUSION_help = \
        "Enable or disable non-linear extrusion compensation using " \
        "the provided polynomial coefficients. Subsequent toolhead moves " \
        "in the positive direction will be affected when relative " \
        "extrusion distances are used."
    def cmd_ENABLE_NONLINEAR_EXTRUSION(self, gcmd):
        value = gcmd.get("COEFFICIENTS", None)
        if value:
            try:
                coefs = ast.literal_eval(value)
            except ValueError as e:
                coefs = None
            if coefs is None or type(coefs) not in (list, tuple):
                gcmd.respond_raw("!! Coefficients must be a list of floats, not %s (check syntax?)\n" % (type(coefs).__name__, ))
                return

            logging.info("Setting non-linear extrusion coefficients: %s" % (repr(coefs), ))
            self.coefficients = coefs

        enabled = gcmd.get_int("ENABLE", 1) == 1
        if enabled and self.coefficients is None:
            gcmd.respond_raw(f"!! Cannot enable non-linear extrusion without configuring coefficients first.\n")
            return

        self.enabled = enabled
        if enabled:
            gcmd.respond_info(f"Non-lienar extrusion enabled with coefficients {self.coefficients}.")
        else:
            gcmd.respond_info(f"Non-lienar extrusion disabled.")

    def _handle_ready(self):
        self.gcode_move = self.printer.lookup_object('gcode_move')
        self.toolhead = self.printer.lookup_object('toolhead')
        self.extruder = self.printer.lookup_object(self.extruder_name)

        # Register transform
        old_transform = self.gcode_move.set_move_transform(self, force=True)
        self._next_transform = old_transform

    def _compensated_speed(self, speed):
        if speed == 0.:
            return speed
        new_speed = float(numpy.polynomial.polynomial.polyval(speed, self.coefficients))
        return max(new_speed, speed)

    def _compensated_length(self, e_distance, speed, axes_distance=None):
        if axes_distance:
            # printing move
            speed = min(speed, self.toolhead.max_velocity)
            move_duration = axes_distance / speed
            e_speed = e_distance / move_duration
        else:
            # extrude-only move
            move_duration = e_distance / speed
            e_speed = speed

        new_speed = self._compensated_speed(e_speed)
        new_length = new_speed * move_duration

        if self._logging:
            logging.info(f"compensation for move distance [axes={axes_distance:.4f}mm,E={e_distance:.4f}mm] (new={new_length:.2f}mm) at speed={speed:.2f}mm/s. "\
                        f"extrude speed={e_speed:.2f}mm/s (new={new_speed:.2f}mm/s), duration={move_duration:.4f}s.")
        return new_length, new_speed

    def _can_apply_compensation(self):
        if self.gcode_move is None or self.toolhead is None or self.extruder is None:
            return False

        if not self.enabled:
            return False

        if self.gcode_move.absolute_coord and self.gcode_move.absolute_extrude:
            return False

        if self.toolhead.get_extruder() is not self.extruder:
            return False

        return True

    def move(self, newpos, speed):
        if self._can_apply_compensation():
            oldpos = self.toolhead.commanded_pos
            axes_d = [newpos[i] - oldpos[i] for i in (0, 1, 2, 3)]
            move_d = math.sqrt(sum([d*d for d in axes_d[:2]]))
            extrude_only = (move_d < .000000001)

            extrude_d = axes_d[3]
            if extrude_d > 0.:
                new_length, new_speed = self._compensated_length(extrude_d, speed, axes_distance=(0 if extrude_only else move_d))

                if self._logging:
                    logging.info(f"applying compensation for {'extrude only' if extrude_only else 'printing'} move from {oldpos} to {newpos}.")

                newpos[3] = oldpos[3] + new_length

                if extrude_only:
                    speed = new_speed
        return self._next_transform.move(newpos, speed)

    def get_position(self):
        return self._next_transform.get_position()

def load_config_prefix(config):
    return NonLinearExtrusion(config)
