"""
 Copyright (c) 2021 Alan Yorinks All rights reserved.

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU AFFERO GENERAL PUBLIC LICENSE
 Version 3 as published by the Free Software Foundation; either
 or (at your option) any later version.
 This library is distributed in the hope that it will be useful,f
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 General Public License for more details.

 You should have received a copy of the GNU AFFERO GENERAL PUBLIC LICENSE
 along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

"""

import sys
import time

from telemetrix_rpi_pico_w import telemetrix_rpi_pico_w

"""
Setup a pin for output and fade its intensity
"""

# some globals
# make sure to select a PWM pin
DIGITAL_PIN = 0

# Create a Telemetrix instance.
board = telemetrix_rpi_pico_w.TelemetrixRpiPicoW(ip_address='192.168.2.102')
board.pwm_range(255)

# Set the DIGITAL_PIN as an output pin
board.set_pin_mode_pwm_output(DIGITAL_PIN)

# When hitting control-c to end the program
# in this loop, we are likely to get a KeyboardInterrupt
# exception. Catch the exception and exit gracefully.

try:
    # use raw values for a fade
    for level in range(0, 255, 5):
        board.pwm_write(DIGITAL_PIN, level)
        time.sleep(.001)
    for level in range(255, 0, -5):
        board.pwm_write(DIGITAL_PIN, level)
        time.sleep(.001)

    board.pwm_write(DIGITAL_PIN, 0)

except KeyboardInterrupt:
    board.pwm_write(DIGITAL_PIN, 0)
    board.shutdown()
    exit(0)

board.shutdown()
exit(0)
