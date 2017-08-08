#!/usr/bin/env python
# -*- coding: utf-8 -*-

""" This Module serves as an interface for the interaction board present in the SQUIRREL robot

The board controls the following devices:

    - Head Motor (pan) (-180º ~ 180ª)

    - Neck Motor (pan) (-180º ~ 180ª)

    - Camera Motor (tilt) (-68º ~ 30ª)

    - Door Motor (pan) (TODO)

    - 84 base RGB Led array

    - 4 mouth RGB Led array

"""

from serial import Serial
import binascii

_MOTORS = ["head", "neck", "camera", "door"]


class Controller:
    """Main interface for Serial communication with Squirrel Interaction board Motors"""
    def __init__(self, port="/dev/ttyUSB0", baudrate=115200):
        self.serial = Serial(port, baudrate)
        self.start_base_led_colors(84)

    def __del__(self):
        self.serial.close()

    def reset(self):
        for each_motor in _MOTORS:
            self.move_to(each_motor, 0)

    def move_to(self, motor, degrees):
        try:
            self.serial.reset_input_buffer()
            self.serial.write([
                0x30,
                _MOTORS.index(motor),
                0 if degrees >= 0 else 255,
                degrees & 0xff
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
            return 'Available motors: head, neck, camera, door;'

    def set_mouth_led_colors(self, colors):
        if len(colors) != 12:
            return 'Need RGB values (0-255, 0-255, 0-255) for each Mouth LED (4)'
        colors_ = map(limit_color, colors)
        self.serial.reset_input_buffer()
        self.serial.write([0x3A] + colors_)
        return self.check_response(4, 0x3A)

    def start_base_led_colors(self, number_of_leds):
        self.serial.write([0x34, number_of_leds])
        return self.check_response(4, 0x34)

    def set_base_led_colors(self, colors):
        """sets the rgb value for each base led (84 max)"""
        if len(colors) != 252:
            return 'Need RGB values (0-255, 0-255, 0-255) for each Mouth LED (4)'
        colors_ = map(limit_color, colors)
        self.serial.reset_input_buffer()
        self.serial.write([0x35] + colors_)
        return self.check_response(4, 0x35)

    def check_response(self, number_of_bytes_to_read, should_be):
        response = bytearray()
        response.extend(self.serial.read(number_of_bytes_to_read))
        return response[0] == should_be


def limit_color(color_value):
    if color_value > 255:
        return 255
    if color_value < 0:
        return 0
    return color_value



