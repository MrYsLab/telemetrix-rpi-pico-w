"""
 Copyright (c) 2022 Alan Yorinks All rights reserved.

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
Monitor a potentiometer connected to ADC2 and the internal reference voltage
"""

# Setup a pin for analog input and monitor its changes
# adc numbers for sensors
POTENTIOMETER = 2
ADC_REF_VOLTAGE = 3

# Callback data indices
CB_PIN_MODE = 0
CB_PIN = 1
CB_VALUE = 2
CB_TIME = 3


def the_callback(data):
    """
    A callback function to report data changes.
    This will print the pin number, its reported value and
    the date and time when the differential is exceeded
    :param data: [report_type, ADC#, current reported value, timestamp]
    """
    date = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(data[CB_TIME]))
    if data[CB_PIN] == POTENTIOMETER:
        print(f'Potentiometer value: {data[CB_VALUE]} at {date}')
    if data[CB_PIN] == ADC_REF_VOLTAGE:
        print(f'Reference voltage: {data[CB_VALUE]} at {date}')


def analog_in(my_board):
    """
     This function establishes monitors changes in analog
     inputs for the internal cpu temperature and reference
     voltages, as well as a potentiometer connected to ADC2

     :param my_board: a pymata4 instance
     """

    # set the pin mode

    my_board.set_pin_mode_analog_input(POTENTIOMETER, differential=10,
                                       callback=the_callback)
    my_board.set_pin_mode_analog_input(ADC_REF_VOLTAGE, differential=10,
                                       callback=the_callback)

    print('Enter Control-C to quit.')
    try:
        while True:
            time.sleep(5)
    except KeyboardInterrupt:
        board.shutdown()
        sys.exit(0)


board = telemetrix_rpi_pico_w.TelemetrixRpiPicoW()

try:
    analog_in(board)
except KeyboardInterrupt:
    board.shutdown()
    sys.exit(0)
