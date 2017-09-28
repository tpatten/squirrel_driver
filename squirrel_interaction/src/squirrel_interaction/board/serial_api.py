#!/usr/bin/env python
# -*- coding: utf-8 -*-

""" This Module serves as an interface for the interaction board present in the SQUIRREL robot

The board controls the following devices:

    - Head Motor (pan) (-180º ~ 180ª) - head_controller

    - Neck Motor (pan) (-180º ~ 180ª) - neck_pan_controller

    - Camera Motor (tilt) (-68º ~ 30ª) - neck_tilt_controller

    - Door Motor (pan) (TODO) - door_controller

    - 84 base RGB Led array

    - 4 mouth RGB Led array

"""

from serial import Serial
import binascii

_MOTORS = ["head", "neck", "camera", "door"]
_NUMBER_OF_BASE_LEDS = 42
_NUMBER_OF_MOUTH_LEDS = 4


class Controller:
    """ Main interface for Serial communication with Squirrel Interaction board """
    def __init__(self, port="/dev/ttyUSB0", baudrate=115200):
        self.serial = Serial(port, baudrate)
        self.start_base_led_colors(_NUMBER_OF_BASE_LEDS)

    def __del__(self):
        self.serial.close()

    def reset(self):
        for each_motor in _MOTORS:
            self.move_to(each_motor, 0)

    def move_to(self, motor, degrees):
        if not _valid_destination(motor, degrees):
            return 'Invalid destination %d for %s' % (degrees, motor)
        try:
            message = bytearray(2)
            message[0] = (degrees >> 8) & 0xff  # simple bitwise operations to split an integer into 2 bytes
            message[1] = degrees & 0xff
            self.serial.reset_input_buffer()
            self.serial.write([
                0x30,
                _MOTORS.index(motor),
                message[0],
                message[1]
            ])
            response = bytearray()
            response.extend(self.serial.read(5))
            return response[0] == 0x30

        except ValueError:
            return 'Available motors: head, neck, camera, door;'

    def get_position(self, motor):
        try:
            self.serial.reset_input_buffer()
            self.serial.write([
                0x52,
                _MOTORS.index(motor)
            ])
            response = bytearray()
            response.extend(self.serial.read(7))
            return - (response[2] - response[3])

        except ValueError:
            return 'Available motors: head, neck, camera;'

    def set_mouth_led_colors(self, colors):
        if len(colors) != _NUMBER_OF_MOUTH_LEDS * 3:
            return 'Need RGB values (0-255, 0-255, 0-255) for each Base LED (%d)' % _NUMBER_OF_MOUTH_LEDS
        colors_ = map(_limit_color, colors)
        self.serial.reset_input_buffer()
        self.serial.write([0x3A] + colors_)
        return self._check_response(4, 0x3A)

    def start_base_led_colors(self, number_of_leds):
        self.serial.write([0x34, number_of_leds])
        return self._check_response(4, 0x34)

    def set_base_led_colors(self, colors):
        """sets the rgb value for each base led (42 max)"""
        if len(colors) != _NUMBER_OF_BASE_LEDS * 3:
            return 'Need RGB values (0-255, 0-255, 0-255) for each Base LED (%d)' % _NUMBER_OF_BASE_LEDS
        colors_ = map(_limit_color, colors)
        self.serial.reset_input_buffer()
        self.serial.write([0x35] + colors_)
        return self._check_response(4, 0x35)

    def get_door_status(self):
        self.serial.reset_input_buffer()
        self.serial.write([
            0x55,
            3
        ])
        response = bytearray()
        response.extend(self.serial.read(6))
        if response[2] == 0x01:
            return "OPEN"
        if response[2] == 0x10:
            return "CLOSED"
        return "AJAR"

    def _check_response(self, number_of_bytes_to_read, should_be):
        response = bytearray()
        response.extend(self.serial.read(number_of_bytes_to_read))
        return response[0] == should_be


def _limit_color(color_value):
    if color_value > 255:
        return 255
    if color_value < 0:
        return 0
    return color_value


def _valid_destination(motor, degrees):
    if motor in ['head', 'neck']:
        return -180 <= degrees <= 180
    if motor == 'camera':
        return -60 <= degrees <= 30
    if motor == 'door':
        return -20000 <= degrees <= 30000
    return False

