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

import asyncio
import sys
import time

from telemetrix_rpi_pico_w_aio import telemetrix_rpi_pico_w_aio

"""
Monitor a digital input pin
"""

"""
Setup a pin for digital input and monitor its changes
"""

# Set up a pin for analog input and monitor its changes
DIGITAL_PIN = 5  # GPIO pin number

# Callback data indices
CB_PIN_MODE = 0
CB_PIN = 1
CB_VALUE = 2
CB_TIME = 3


async def the_callback(data):
    """
    A callback function to report data changes.
    This will print the pin number, its reported value and
    the date and time when the change occurred

    :param data: [pin_mode, pin, current reported value, timestamp]
    """
    date = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(data[CB_TIME]))
    print(f'Pin: {data[CB_PIN]} Value: {data[CB_VALUE]} Time Stamp: {date}')


async def digital_in(my_board, pin):
    """
     This function establishes the pin as a
     digital input. Any changes on this pin will
     be reported through the call back function.

     :param my_board: a telemetrix_rpi_pico_aio instance
     :param pin: Pico GPIO  number
     """

    # set the pin mode
    await my_board.set_pin_mode_digital_input(pin, the_callback)
    print('Reporting enabled for 5 seconds.')
    await asyncio.sleep(5)
    print('Disabling reporting for DIGITAL_INPUT_PIN 3 seconds.')
    await my_board.disable_digital_reporting(pin)
    await asyncio.sleep(3)
    print('Re-enabling reporting for DIGITAL_INPUT_PIN.')
    await my_board.enable_digital_reporting(pin)
    while True:
        try:
            await asyncio.sleep(.001)
        except KeyboardInterrupt:
            await board.shutdown()
            sys.exit(0)

# get the event loop
loop = asyncio.new_event_loop()
asyncio.set_event_loop(loop)

# instantiate TelemetrixRpiPicoWAio
board = telemetrix_rpi_pico_w_aio.TelemetrixRpiPicoWAio(ip_address='192.168.2.102', loop=loop)

try:
    # start the main function
    loop.run_until_complete(digital_in(board, DIGITAL_PIN))
except (KeyboardInterrupt, RuntimeError) as e:
    loop.run_until_complete(board.shutdown())
    sys.exit(0)
