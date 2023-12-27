# High Resolution Filament Sensor
#
# Copyright (C) 2023 Francois Chagnon <fc@francoischagnon.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import struct
import logging
import typing
from . import bus, filament_switch_sensor, tmc_uart

DEFAULT_I2C_TARGET_ADDR = 0x40
DEFAULT_I2C_SPEED = 100000

CHECK_RUNOUT_TIMEOUT = .100 # read sensor value at this interval

class MagnetState:
    """" State of the rotary magnet encoder inside the sensor. """
    NOT_DETECTED = 1
    TOO_WEAK = 2
    TOO_STRONG = 3
    DETECTED = 4

    VALUES : dict[int, str] = {
        NOT_DETECTED: "not detected",
        TOO_WEAK: "too weak",
        TOO_STRONG: "too strong",
        DETECTED: "detected",
    }

    def __init__(self, value : int):
        self.value = value

    def __str__(self):
        return MagnetState.VALUES.get(self.value, "unknown")

    def __repr__(self):
        return "%s(value=%s)" % (self.__class__.__name__, repr(self.value))

class SensorRegister:
    """ Enum of registers that can be read from the sensor. """
    ALL = 0x10
    MAGNET_STATE = 0x21
    FILAMENT_PRESENCE = 0x22
    FULL_TURNS = 0x23
    ANGLE = 0x24

class SensorEvent:
    """ A point-in-time reading from the sensor, which holds calculated
    values as well as additional printer state. """

    def __init__(self, eventtime : float, distance : float, epos : float):
        # eventtime at which this sensor reading was taken
        self.eventtime = eventtime

        # distance since previous reading from the sensor
        self.distance = distance

        # estimated extruder position at this eventtime
        self.epos = epos

    def __repr__(self):
        return "%s(eventtime=%s, distance=%s, epos=%s)" % (
            self.__class__.__name__, self.eventtime, self.distance, self.epos)

class SensorUART(tmc_uart.MCU_TMC_uart_bitbang):
    """ Class for reading from the sensor via Klipper's native TMC uart driver. """

    def _remove_serial_bits(self, data : str) -> bytearray:
        """ Remove serial start and stop bits to a message in a bytearray. """
        mval = pos = 0
        for d in bytearray(data):
            mval |= d << pos
            pos += 8

        pos = 0
        res = bytearray()
        for i in range((len(data) * 8) // 10):
           shift = (i * 10) + 1
           res.append((mval >> shift) & 0xff)
        return res

    def _decode_read(self, reg : int, data : str) -> typing.Optional[bytearray]:
        """ Extract a uart read response message and returns the decoded message.
        Returns None when message cannot be verified. """
        decoded = self._remove_serial_bits(data)
        if len(decoded) < 5:
            return None
        if decoded[-1] != self._calc_crc8(decoded[:-1]):
            return None
        if decoded[0] != 0x05 and decoded[1] != 0xff:
            logging.warning("Received wrong message prefix: %s" % (decoded.hex(), ))
            return None
        if decoded[2] != reg:
            logging.warning("Received response for reg %02x (expected %02x)" % (decoded[2], reg))
            return None
        return decoded[3:-1]

    def reg_read(self, _instance_id, addr : int, reg : int, reg_length : int = 4) -> typing.Optional[bytearray]:
        """ Read a single register and returns a bytearray. """
        msg = self._encode_read(0xf5, addr, reg)
        read_length = (((4 + reg_length) * 10) + 7) // 8
        params = self.tmcuart_send_cmd.send([self.oid, msg, read_length])
        return self._decode_read(reg, params['read'])

class SensorRotationHelper:
    """ Helper class used to convert raw point-in-time readings from the sensor
    into an angular change since the previous reading. """

    def __init__(self, resolution : int, ignore_bits : int):
        self.resolution = resolution
        self.ignore_bits = ignore_bits

        # Maximum and minimum values that can be obtained from the sensor
        self.angle_max_value = (1 << resolution) - 1
        self.angle_min_value = (1 << ignore_bits)

        # Value used to mask lower bits of the raw angle value read from the printer.
        self.mask = (1 << ignore_bits) - 1

        # Number of full turns
        self._turns = 0

        # Raw angle value, between zero and angle_max_value
        self._angle = 0

        # The cumulative angle value including all full turns
        self._absolute_angular_position = 0

        # Angle change since the value was last reset
        self._angle_change = 0

    @property
    def angular_resolution(self):
        """ Returns the resolution given the number of bits the sensor returns. """
        return (self.angle_min_value / float(self.angle_max_value) * 360.)

    def absolute_angular_position(self):
        """ Returns the cumulative number of degrees turned since the sensor was booted up. """
        return ((self._absolute_angular_position & ~self.mask) / float(self.angle_max_value) * 360.)

    def angle_change(self):
        """ The change in angle since clear_angle_change was last called. """
        return ((self._angle_change & ~self.mask) / float(self.angle_max_value) * 360.)

    def clear_angle_change(self):
        """ Sets the current angle change to 0. """
        self._angle_change = 0

    def update_raw(self, turns, angle):
        """ Set the current number of full turns and the current angle,
            compute the change in angular position. """
        self._turns = turns
        self._angle = angle

        # calculate angle change since last update
        abs_angular_pos = (turns * self.angle_max_value) + angle
        angle_change = abs_angular_pos - self._absolute_angular_position
        self._absolute_angular_position = abs_angular_pos

        # keep track of cumulative change since value was last reset
        self._angle_change += angle_change

class MotionDirection:
    """" Wrapper for the motion direction. """
    def __init__(self, distance : float):
        self.distance = distance

    def __str__(self):
        if not self.distance:
            return "idle"
        elif self.distance > 0:
            return "extruding"
        else:
            return "reversing"

    def __repr__(self):
        return f"{self.__class__.__name__}(distance={self.distance})"

class CommandedMove:
    """ Represents a movement that was commanded to the printer around a given eventtime.
    Also holds the sensor position and the estimated extruder position around this eventtime.

    When the printer is ordered to move its extruder motor by a certain amount, we know
    the commanded (final) position ahead of time before the motor actually starts moving.
    As we read data from the sensor, the measured distance between each sensor reading is
    accumulated until the move is completed.
    """

    def __init__(self, eventtime, pos, last_epos, epos):
        # eventtime when this move was started
        self.eventtime : float = eventtime

        # Sensor position when the move was started
        self.pos : float = pos

        # Last known toolhead position before the move was started.
        # This can be a reading up to `CHECK_RUNOUT_TIME` seconds in the past.
        self.last_epos : float = last_epos

        # The commanded position, i.e. the final position once this
        # move is done, possibly in the future.
        self.epos : float = epos

        # Expected distance travelled once this move is done.
        self.distance : float = epos - last_epos

        # False while the move is happening, True once the printer is stopped or another move starts.
        self.ended : bool = False

        # Running total of 'distance' for all `SensorEvent`, while the move is happening
        self.measured_distance : float = 0.0

        # All `SensorEvent` objects that happened during this move
        # self.sensor_events : list[SensorEvent] = []
        self.first_event : typing.Optional[SensorEvent] = None
        self.last_event : typing.Optional[SensorEvent] = None
        self.first_motion_event : typing.Optional[SensorEvent] = None
        self.last_motion_event : typing.Optional[SensorEvent] = None

    def add_sensor_event(self, event):
        self.measured_distance += event.distance
        # self.sensor_events.insert(0, event)
        if self.first_event is None:
            self.first_event = event
        if event.distance > 0:
            if self.first_motion_event is None:
                self.first_motion_event = event
            self.last_motion_event = event
        self.last_event = event

    @property
    def last_motion_eventtime(self) -> typing.Optional[float]:
        """ The most recent `eventtime` at which the sensor had a non-zero `distance` reading. """
        if self.last_motion_event:
            return self.last_motion_event.eventtime

    @property
    def first_motion_eventtime(self) -> typing.Optional[float]:
        """ The most recent `eventtime` at which the sensor had a non-zero `distance` reading. """
        if self.first_motion_event:
            return self.first_motion_event.eventtime

    def has_stopped_moving(self, duration=0.1) -> bool:
        if not (e := self.last_motion_eventtime):
            return False
        diff = self.last_event.eventtime - e
        if e == 0.0:
            return False
        return e > duration

    @property
    def duration(self) -> typing.Optional[float]:
        """ The actual duration of the move. """
        if e := self.last_motion_eventtime:
            return e - self.first_motion_eventtime

    @property
    def expected_distance(self) -> typing.Optional[float]:
        """ Returns distance that the extruder is expected to have travelled
        between the start of the move and the most recent sensor reading. """
        if self.first_event:
            return self.last_event.epos - self.last_epos

    @property
    def speed(self) -> typing.Optional[float]:
        """ Returns the average speed during the move (measured distance divided by duration). """
        if d := self.duration:
            return self.measured_distance / d

    @property
    def extrusion_rate(self) -> float:
        """ Returns the extrusion rate, (measured distance over the expected distance). """
        if d := self.expected_distance:
            return self.measured_distance / d
        return 0.0

    @property
    def detected(self) -> bool:
        return self.measured_distance > 0

    @property
    def direction(self) -> str:
        """ Returns which way the extruder is moving given a positive or negative distance value. """
        return MotionDirection(self.measured_distance)

class TriggerOnChange:
    """ Calls a function when the value is changed from False to True, only once. """

    def __init__(self, value : typing.Optional[bool], fn):
        self._value : bool = value
        self._fn = fn

    def set(self, value : bool):
        if value is not self._value:
            self._fn(self._value, value)
        self._value = value

    def __bool__(self):
        return self._value
    __nonzero__=__bool__

class RunoutHelper(filament_switch_sensor.RunoutHelper):
    def __init__(self, config, sensor):
        super().__init__(config)
        self._sensor = sensor

    def cmd_QUERY_FILAMENT_SENSOR(self, gcmd):
        return self._sensor.cmd_QUERY_FILAMENT_SENSOR(gcmd)

class HighResolutionFilamentSensor:
    """ A filament sensor from which we can get extremely accurate position readings. """

    def __init__(self, config):
        self.name = config.get_name().split()[-1]
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.reactor = self.printer.get_reactor()
        self.runout_helper = RunoutHelper(config, self)
        self.estimated_print_time = None

        self.uart = self._lookup_uart_bitbang(config)
        if self.uart:
            self.mcu = self.uart.mcu
        else:
            self.i2c = bus.MCU_I2C_from_config(config, DEFAULT_I2C_TARGET_ADDR, DEFAULT_I2C_SPEED)
            self.mcu = self.i2c.mcu

        self.extruder_name = config.get('extruder')
        self.invert_direction = config.getboolean('invert_direction', False)
        self.rotation_distance = config.getfloat('rotation_distance', minval=0)
        self.underextrusion_max_rate = config.getfloat('underextrusion_max_rate', minval=0.0, maxval=1.0)
        self.underextrusion_period = config.getfloat('underextrusion_period', minval=0.0)
        self.move_evaluation_distance = config.getfloat('move_evaluation_distance', 1, minval=0.0)

        # Printer state
        self._commanded_moves : list[CommandedMove] = []
        self.position = 0.0
        self._is_calibrating = False
        self._is_printing = False
        self._unhealthy = TriggerOnChange(False, self._unhealthy_changed)
        self._runout = TriggerOnChange(False, self._runout_changed)
        self._underextrusion_start_time = None
        self._underextruding = TriggerOnChange(False, self._underextruding_changed)
        self._last_epos = 0.0
        self._status_evaluation_move = None

        # Internal sensor state
        self._magnet_state = MagnetState(0xff)
        self._sensor_connected = TriggerOnChange(None, self._sensor_connected_changed)
        self._filament_present = TriggerOnChange(None, self._filament_present_changed)
        self._rotation_helper = SensorRotationHelper(12, 3) # 12 bits of precision, ignore lower 3 bits

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
        self.printer.register_event_handler('toolhead:set_position',
                self._toolhead_set_position)

        self.gcode.register_mux_command(
            "CALIBRATE_FILAMENT_SENSOR_ROTATION_DISTANCE", "SENSOR", self.name,
            self.cmd_CALIBRATE_FILAMENT_SENSOR_ROTATION_DISTANCE,
            desc=self.cmd_CALIBRATE_FILAMENT_SENSOR_ROTATION_DISTANCE_help)

        self.gcode.register_mux_command(
            "CALIBRATE_MAX_FLOW", "SENSOR", self.name,
            self.cmd_CALIBRATE_MAX_FLOW,
            desc=self.cmd_CALIBRATE_MAX_FLOW_help)

    def cmd_QUERY_FILAMENT_SENSOR(self, gcmd):
        sensor_connected = "connected" if self._sensor_connected else "not connected"
        filament_present = "detected" if self._filament_present else "not detected"
        runout_detected = "detected" if self._runout else "not detected"
        underextruding_detected = "detected" if self._underextruding else "not detected"
        msg = f"Filament Sensor {self.name}:\n"
        msg += f"- sensor {sensor_connected}\n"
        msg += f"- filament {filament_present}\n"
        msg += f"- runout {runout_detected}\n"
        msg += f"- underextrusion {underextruding_detected}\n"
        msg += f"- resolution: {self._rotation_helper.resolution} bits (lower {self._rotation_helper.ignore_bits} bits are ignored)\n"
        msg += f"- smallest detectable angular change: {self._rotation_helper.angular_resolution} degree\n"
        msg += f"- smallest detectable movement: {self._rotation_helper.angular_resolution / 360. * self.rotation_distance} mm\n"
        gcmd.respond_info(msg)

    def _lookup_uart_bitbang(self, config) -> typing.Optional[SensorUART]:
        if not (rx_pin := config.get('uart_rx_pin', None)):
            return
        ppins = config.get_printer().lookup_object("pins")
        rx_pin_params = ppins.lookup_pin(rx_pin, can_pullup=True)
        tx_pin_params = ppins.lookup_pin(config.get('uart_tx_pin'), can_pullup=True)

        if rx_pin_params['chip'] is not tx_pin_params['chip']:
            raise ppins.error("%s uart rx and tx pins must be on the same mcu")

        return SensorUART(rx_pin_params, tx_pin_params, None)

    def _toolhead_set_position(self):
        """ Callback when kinematic position is set for the toolhead.
        This changes the current toolhead position without moving. """
        toolhead = self.printer.lookup_object('toolhead')
        if toolhead.get_extruder() is self.extruder:
            # toolhead position was updated without a move, keep track of this
            self._last_epos = toolhead.get_position()[3]

    def _measured_underextrusion_rate(self, move):
        """ Return a measure of how much we are actually extruding
        compared to how much was expected over the events kept in
        the history buffer. The rate is returned as a value from
        0.0 to 1.0, 0 means the measured rate matches the expected
        rate and 1 meaning no extrusion at all when some was expected. """

        if not move:
            return 0.0
        return min(1, max(-1, 1. - move.extrusion_rate))

    def _combine_moves(self, newer : CommandedMove, older : CommandedMove):
        move = CommandedMove(older.eventtime, older.pos, older.last_epos, newer.epos)
        move.distance = newer.epos - older.last_epos
        move.ended = newer.ended
        move.measured_distance = older.measured_distance + newer.measured_distance
        move.first_event = older.first_event or newer.first_event
        move.first_motion_event = older.first_motion_event or newer.first_motion_event
        move.last_motion_event = newer.last_motion_event or older.last_motion_event
        move.last_event = newer.last_event or older.last_event
        return move

    def _combine_moves_for_distance(self, distance):
        move = None

        for _move in self._commanded_moves:
            if not move:
                move = _move
            else:
                move = self._combine_moves(move, _move)
            if move.distance >= distance:
                break

        return move

    def get_status(self, eventtime):
        move = self._status_evaluation_move
        speed = move.speed if move and not move.ended else None

        return {
            "enabled": bool(self.runout_helper.sensor_enabled),
            "sensor_connected": bool(self._sensor_connected),
            "magnet_state": str(self._magnet_state),
            "filament_detected": bool(self._filament_present),
            "motion": {
                "detected": move.detected if move else False,
                "direction": str(move.direction) if move else "idle",
                "commanded_distance": move.distance if move and not move.ended else 0.0,
                "expected_distance": move.expected_distance if move else 0.0,
                "measured_distance": move.measured_distance if move else 0.0,
                "measured_speed": speed or 0.0,
                "measured_volumetric_flow": self._speed_to_volumetric_flow(speed) if speed else 0.0,
            },
            "underextrusion_rate": self._measured_underextrusion_rate(move) if move and self._is_printing else 0.0,
            "underextrusion_detected": bool(self._underextruding),
            "runout": bool(self._runout),
            "position": self.position,
        }

    def uart_read_reg(self, reg, length, retries=5):
        for _ in range(retries):
            response = self.uart.reg_read(None, 0, reg, length)
            if response:
                break
        if response is None:
            logging.warning("Error reading from uart, no response received or CRC was invalid.")
        else:
            return response

    def uart_read_reg4(self, reg):
        if data := self.uart_read_reg(reg, 4):
            val, = struct.unpack('<l', data)
            return val

    def uart_read_reg1(self, reg):
        if data := self.uart_read_reg(reg, 1):
            val, = struct.unpack('<B', data)
            return val

    def i2c_read_reg(self, reg, length):
        params = self.i2c.i2c_read([reg], length)
        return bytearray(params['response'])

    def _get_extruder_pos(self, eventtime=None):
        """ Find the estimated extruder position at the given eventtime. """

        if self.estimated_print_time is None:
            return None

        if eventtime is None:
            eventtime = self.reactor.monotonic()

        print_time = self.estimated_print_time(eventtime)
        return self.extruder.find_past_position(print_time)

    def _inspect_commanded_move(self, eventtime):
        """ Check if the commanded position of the extruder has changed and
        keep track of commanded moves. """

        toolhead = self.printer.lookup_object('toolhead')
        if toolhead.get_extruder() is not self.extruder:
            return

        epos = toolhead.get_position()[3]
        distance = epos - self._last_epos
        if distance > 0:
            previous_event = None
            if len(self._commanded_moves) > 0:
                move = self._commanded_moves[0]
                move.ended = True
                previous_event = move.last_event

            move = CommandedMove(eventtime, self.position, self._last_epos, epos)
            self._commanded_moves.insert(0, move)
            self._last_epos = move.epos

            # insert last event from previous move (i.e. the previous sensor reading)
            # into the current move, to get an accurate speed without discontinuity.
            if previous_event:
                move.add_sensor_event(previous_event)

        while len(self._commanded_moves) > 100:
            self._commanded_moves.pop()

    def _sensor_connected_changed(self, old_value, new_value):
        logging.info(f"{self.name}: 'sensor_connected' changed from {old_value} to {new_value}")

        if old_value is None:
            return
        if new_value:
            self._respond_info("Reconnected")
        else:
            self._respond_error("No longer connected or data cannot be read")

    def _filament_present_changed(self, old_value, new_value):
        logging.info(f"{self.name}: 'filament_present' changed from {old_value} to {new_value}")

        if old_value is None:
            return
        if new_value:
            self._respond_info("Filament present")
        else:
            self._respond_error("Filament not present")

    def _update_state_from_sensor(self):
        """ Read data from sensor and sets our internal state to match. """

        eventtime = self.reactor.monotonic()
        self._inspect_commanded_move(eventtime)

        if self.uart:
            magnet_state = self.uart_read_reg1(SensorRegister.MAGNET_STATE)
            filament_presence = self.uart_read_reg1(SensorRegister.FILAMENT_PRESENCE)
            full_turns = self.uart_read_reg4(SensorRegister.FULL_TURNS)
            angle = self.uart_read_reg4(SensorRegister.ANGLE)

            self._sensor_connected.set(not (magnet_state is None or
                                          filament_presence is None or
                                          full_turns is None or
                                          angle is None))

            if not self._sensor_connected:
                return
        else:
            data = self.i2c_read_reg(SensorRegister.ALL, 10)
            if len(data) != 10:
                self._sensor_connected.set(False)
                logging.warning("Update from sensor failed, expected 10 bytes but got %d: '%s'" % (len(data), data.hex(), ))
                return

            magnet_state, filament_presence, full_turns, angle = struct.unpack('<BBll', data)

            self._sensor_connected.set(magnet_state != 0xff)
            if not self._sensor_connected:
                return

        self._magnet_state = MagnetState(magnet_state)
        self._filament_present.set(filament_presence == 1)

        self._rotation_helper.update_raw(full_turns, angle)
        angle_change = self._rotation_helper.angle_change()

        if angle_change != 0.0:
            self._rotation_helper.clear_angle_change()

        if self.invert_direction:
            angle_change = angle_change * -1

        distance = self.rotation_distance * angle_change / 360.
        self.position += distance

        event = SensorEvent(eventtime, distance, self._get_extruder_pos(eventtime))
        if len(self._commanded_moves) > 0:
            move = self._commanded_moves[0]
            if not move.ended:
                move.add_sensor_event(event)

    def _handle_ready(self):
        """ Callback when printer becomes ready. """

        logging.info("[%s] ready" % (self.__class__.__name__, ))

        self.extruder = self.printer.lookup_object(self.extruder_name)
        self.estimated_print_time = self.printer.lookup_object('mcu').estimated_print_time
        self.reactor.update_timer(self._sensor_update_timer, self.reactor.NOW)

    def _handle_printing(self, print_time):
        """ Callback when printing starts. """

        logging.info("[%s] printing" % (self.__class__.__name__, ))
        self._runout.set(False)
        self._underextrusion_start_time = None
        self._underextruding.set(False)
        self._is_printing = True
        self._commanded_moves = [] # clear move queue.
        self._status_evaluation_move = None

    def _handle_not_printing(self, print_time):
        """ Callback when printing is finished. """

        logging.info("[%s] not printing" % (self.__class__.__name__, ))
        self._is_printing = False

        self._mark_commanded_moves_ended()

        return

    def _mark_commanded_moves_ended(self):
        for move in self._commanded_moves:
            move.ended = True

    def _respond_error(self, msg):
        """ Print and error to the gcode console. """
        msg = f"{self.name}: {msg}"
        logging.warning(msg)
        lines = msg.strip().split('\n')
        if len(lines) > 1:
            self.gcode.respond_info("\n".join(lines), log=False)
        self.gcode.respond_raw('!! %s' % (lines[0].strip(),))

    def _respond_info(self, msg, log=False):
        self.gcode.respond_info(f"{self.name}: {msg}", log)

    def _exec_gcode(self, prefix, template):
        """ Execute the given gcode with error handling. """
        try:
            self.gcode.run_script(prefix + template.render() + "\nM400")
        except Exception:
            logging.exception("Script running error")
        self.runout_helper.min_event_systime = self.reactor.monotonic() + self.runout_helper.event_delay

    def _runout_event_handler(self, eventtime):
        """ Call the runout code and optionally pause the print. """
        pause_prefix = ""
        if self.runout_helper.runout_pause:
            # Pausing from inside an event requires that the pause portion
            # of pause_resume execute immediately.
            pause_resume = self.printer.lookup_object('pause_resume')
            pause_resume.send_pause_command()
            pause_prefix = "PAUSE\n"
            self.printer.get_reactor().pause(eventtime + self.runout_helper.pause_delay)
        self._exec_gcode(pause_prefix, self.runout_helper.runout_gcode)

    def _is_sensor_healthy(self):
        """ The sensor is 'unhealthy' when it stops responding, or when the magnet
        is too far from the magnetic rotary encoder."""
        return self._sensor_connected and \
            self._magnet_state.value == MagnetState.DETECTED

    def _sensor_unhealthy_reason(self) -> str:
        """ Returns a reason for the sensor being unhealthy for displaying in error messages. """
        if not self._sensor_connected:
            return "no data from sensor"
        if self._magnet_state.value != MagnetState.DETECTED:
            return "magnet %s" % (str(self._magnet_state), )
        return "unknown reason"

    def _is_runout_condition(self):
        """ Checks whether there is a runout condition, either immediately when
        the filament is not detected, or after the configured period of time when
        underextruding. """

        if not self._filament_present:
            return True

        rate = self._measured_underextrusion_rate(self._status_evaluation_move)
        if rate > self.underextrusion_max_rate:
            if self._underextrusion_start_time is None:
                self._underextrusion_start_time = self.reactor.monotonic()
                # self._respond_error("Detected %.2f%% underextrusion starting at %.2f" %
                #                     (rate * 100, self._underextrusion_start_time))
                return False
            elif self._underextrusion_start_time + self.underextrusion_period < self.reactor.monotonic():
                self._underextruding.set(True)
                return True
        elif self._underextrusion_start_time is not None:
            self._underextruding.set(False)
            self._underextrusion_start_time = None

        return False

    def _unhealthy_changed(self, old_value, new_value):
        logging.info(f"{self.name}: 'unhealthy' changed from {old_value} to {new_value}")
        if new_value is False:
            return

        if new_value is True:
            self._respond_error("Unhealthy (%s)" %
                                (self._sensor_unhealthy_reason(), ))
        else:
            self._respond_info("Became healthy again")

    def _runout_changed(self, old_value, new_value):
        logging.info(f"{self.name}: 'runout' changed from {old_value} to {new_value}")

    def _underextruding_changed(self, old_value, new_value):
        logging.info(f"{self.name}: 'underextruding' changed from {old_value} to {new_value}")

        if new_value:
            self._respond_error("Detected underextrusion for over %.2fs" %
                                (self.underextrusion_period, ))
        else:
            self._respond_info("Underextrusion cleared after %.2fs" %
                                    (self.reactor.monotonic() - self._underextrusion_start_time))

    def _check_print_issues(self, eventtime):
        """ Call runout code when print issues are detected. """

        if not self._is_printing or self._is_calibrating:
           return

        if not self._is_sensor_healthy():
            if not self._unhealthy:
                self._unhealthy.set(True)
                self._runout_event_handler(eventtime)
            return
        else:
            self._unhealthy.set(False)

        if self._is_runout_condition():
            if not self._runout:
                self._runout.set(True)
                self._runout_event_handler(eventtime)
        else:
            # runout restored
            self._runout.set(False)

    def _sensor_update_event(self, eventtime):
        """ Periodic timer to fetch sensor data and update internal state. """
        self._update_state_from_sensor()
        self._status_evaluation_move = self._combine_moves_for_distance(self.move_evaluation_distance)

        if eventtime >= self.runout_helper.min_event_systime and self.runout_helper.sensor_enabled:
            self._check_print_issues(eventtime)

        return eventtime + CHECK_RUNOUT_TIMEOUT

    def _build_extrude_cmd(self, requested_distance, speed):
        cmd = "M83\n"

        r = 0
        while r < requested_distance:
            remaining_distance = (requested_distance - r)
            if remaining_distance > (self.extruder.max_e_dist / 2):
                distance = self.extruder.max_e_dist / 2
            else:
                distance = remaining_distance
            cmd += "G1 E%.2f F%d\n" % (distance, speed)
            r += distance

        cmd += "M400\n" # wait for move to finish
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

    def _combine_all_commanded_moves(self):
        move = None

        for _move in self._commanded_moves:
            if not move:
                move = _move
            else:
                move = self._combine_moves(move, _move)

        return move

    def _collect_calibration_data(self, gcmd, distance, speed, num_runs):
        self._is_calibrating = True

        runs = []
        for index in range(num_runs):
            data = {}
            runs.append(data)

            self._commanded_moves = [] # clear move queue.
            data['expected_speed'] = speed / 60.

            # extrude some filament
            gcmd.respond_info("Extruding %.2fmm (at speed=%.2fmm/s, flow=%.2fmm³/s)... %d/%d" %
                              (distance, speed / 60., self._speed_to_volumetric_flow(speed / 60.), index + 1, num_runs))
            cmd = self._build_extrude_cmd(distance, speed)
            try:
                self.gcode.run_script_from_command(cmd)
            except Exception as e:
                gcmd.respond_raw(f"!! Error running extrude command: {e}\n")
                return

            # Give time for everything to settle
            t = self.reactor.monotonic()
            while not self._commanded_moves[0].has_stopped_moving() and (self.reactor.monotonic() - t) < 3:
                self.reactor.pause(self.reactor.monotonic() + 0.1)
            self._mark_commanded_moves_ended()

            move = self._combine_all_commanded_moves()
            data['expected_distance'] = move.expected_distance
            data['actual_distance'] = move.measured_distance
            data['actual_duration'] = move.duration

            if not move.measured_distance or move.duration is None:
                gcmd.respond_raw(f"!! Extruder did not move, stopping...\n")
                break

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
        if runs:
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
        return (self.extruder.filament_area * speed) if speed else 0.0

    def _avg(self, array, key):
        values = [d[key] for d in array if d[key] is not None]
        if len(values) == 0:
            return 0
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
            if not runs:
                self._turn_off_heater(gcmd)
                return
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

        cutoff = gcmd.get_float("MIN_EXTRUSION_RATE", 98.)

        stats = []
        for index in range(len(runs)):
            data = runs[index]

            expected_speed = data['expected_speed']
            expected_distance = data['expected_distance']
            expected_flow = self._speed_to_volumetric_flow(expected_speed)
            actual_distance = data['actual_distance']
            move_duration = data['actual_duration']

            move_speed = (actual_distance / move_duration) if move_duration else 0.0
            move_flow = self._speed_to_volumetric_flow(move_speed)
            percent_flow = (move_flow / expected_flow * 100.) if expected_flow else 0.0

            if percent_flow >= cutoff:
                suggested_max_flow = self._speed_to_volumetric_flow(expected_speed)

            stats.append("Run %d/%d - requested=%.2fmm (at speed=%.2fmm/s, flow=%.2fmm³/s), measured=%.2fmm in %.2f seconds (at speed=%.2fmm/, flow=%.2fmm³/s) = %.2f%% vol. flow" %
                         (index + 1, len(runs), expected_distance, expected_speed, expected_flow,
                          actual_distance, move_duration, move_speed, move_flow, percent_flow))
        gcmd.respond_info("\n".join(stats))

        if suggested_max_flow is None:
            gcmd.respond_info("None of the measured extrusion moves have resulted in a vol. flow above %.2f%% of the requested flow, " \
                              "perhaps you should calibrate the sensor e-steps or double-check your hardware?" % (cutoff, ))
        else:
            gcmd.respond_info("Suggested maximum volumetric flow for measured vol. flow above %.2f%% of expected value: %.2f" % (cutoff, suggested_max_flow, ))

def load_config_prefix(config):
    return HighResolutionFilamentSensor(config)
