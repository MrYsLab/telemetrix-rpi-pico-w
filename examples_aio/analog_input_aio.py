"""
 Copyright (c) 2021 Alan Yorinks All rights reserved.

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU AFFERO GENERAL PUBLIC LICENSE
 Version 3 as published by the Free Software Foundation; either
 or (at your option) any later version.
 This library is distributed in the hope that it will be useful,
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
This file demonstrates analog input using both callbacks and
polling. Time stamps are provided in both "cooked" and raw form
"""

# Set up a pin for analog input and monitor its changes
ANALOG_PIN = 2  # adc port number

# Callback data indices
CB_PIN_MODE = 0
CB_PIN = 1
CB_VALUE = 2
CB_TIME = 3


async def the_callback(data):
    """
    A callback function to report data changes.

    :param data: [pin_mode, pin, current_reported_value,  timestamp]
    """

    formatted_time = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(data[CB_TIME]))
    print(f'Analog Call Input Callback: pin={data[CB_PIN]}, '
          f'Value={data[CB_VALUE]} Time={formatted_time} '
          f'(Raw Time={data[CB_TIME]})')


async def analog_in(my_board, pin):
    """
    This function establishes the pin as an
    analog input. Any changes on this pin will
    be reported through the call back function.

    :param my_board: a TelemetrixRpiPicoWAio instance

    :param pin: GPIO pin number
    """
    await my_board.set_pin_mode_analog_input(pin, differential=10, callback=the_callback)

    # run forever waiting for input changes
    try:
        while True:
            await asyncio.sleep(.001)

    except KeyboardInterrupt:
        await my_board.shutdown()
        sys.exit(0)


# get the event loop
loop = asyncio.new_event_loop()
asyncio.set_event_loop(loop)

# instantiate telemetrix
board = telemetrix_rpi_pico_w_aio.TelemetrixRpiPicoWAio(ip_address='192.168.2.102',
                                                        loop=loop)

try:
    # start the main function
    loop.run_until_complete(analog_in(board, ANALOG_PIN))
    loop.run_until_complete(board.shutdown())

except (KeyboardInterrupt, RuntimeError) as e:
    loop.run_until_complete(board.shutdown())
    sys.exit(0)
