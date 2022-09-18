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
Monitor a digital input pin with pullup enabled
"""

"""
Setup a pin for digital input and monitor its changes
"""

# Callback data indices
CB_PIN_MODE = 0
CB_PIN = 1
CB_VALUE = 2
CB_TIME = 3

DIGITAL_INPUT_PIN = 5


def the_callback(data):
    """
    A callback function to report data changes.
    This will print the pin number, its reported value and
    the date and time when the change occurred

    :param data: [pin mode, pin, current reported value, pin_mode, timestamp]
    """
    date = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(data[CB_TIME]))
    print(f'Report Type: {data[CB_PIN_MODE]} Pin: {data[CB_PIN]} '
          f'Value: {data[CB_VALUE]} Time Stamp: {date}')


board = telemetrix_rpi_pico_w.TelemetrixRpiPicoW(ip_address='192.168.2.102')

try:
    print('Reporting enabled for 5 seconds.')
    time.sleep(5)
    print('Disabling reporting for DIGITAL_INPUT_PIN 3 seconds.')
    board.disable_digital_reporting(DIGITAL_INPUT_PIN)
    time.sleep(3)
    print('Re-enabling reporting for DIGITAL_INPUT_PIN.')
    board.enable_digital_reporting(DIGITAL_INPUT_PIN)
    while True:
        time.sleep(5)

except KeyboardInterrupt:
    board.shutdown()
    sys.exit(0)
