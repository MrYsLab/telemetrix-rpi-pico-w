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

from telemetrix_rpi_pico_w_aio import telemetrix_rpi_pico_w_aio
"""
Fade using a range of 255
"""

DIGITAL_PIN = 0


async def fade(my_board, pin):
    await my_board.pwm_range(255)
    await my_board.set_pin_mode_pwm_output(pin)
    # When hitting control-c to end the program
    # in this loop, we are likely to get a KeyboardInterrupt
    # exception. Catch the exception and exit gracefully.

    try:
        # use raw values for a fade
        for level in range(0, 255, 5):
            await my_board.pwm_write(pin, level)
            await asyncio.sleep(.01)
        for level in range(255, 0, -5):
            await my_board.pwm_write(pin, level)
            await asyncio.sleep(.01)

        await my_board.pwm_write(pin, 0)

    except KeyboardInterrupt:
        await board.pwm_write(pin, 0)
        await board.shutdown()
        exit(0)

# get the event loop
loop = asyncio.new_event_loop()
asyncio.set_event_loop(loop)

# instantiate telemetrix
board = telemetrix_rpi_pico_w_aio.TelemetrixRpiPicoWAio(ip_address='192.168.2.102',
                                                        loop=loop)

try:
    # start the main function
    loop.run_until_complete(fade(board, DIGITAL_PIN))
    loop.run_until_complete(board.shutdown())

except KeyboardInterrupt:
    loop.run_until_complete(board.shutdown())
    sys.exit(0)
