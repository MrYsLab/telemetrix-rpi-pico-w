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
Attach a pin to a servo and move it about.
"""

# some globals
SERVO_PIN = 0


async def move_servo(the_board):
    try:
        await the_board.set_pin_mode_servo(SERVO_PIN, 1000, 2000)
        time.sleep(.2)
        await the_board.servo_write(SERVO_PIN, 90)
        time.sleep(1)
        await the_board.servo_write(SERVO_PIN, 0)
        time.sleep(1)
        await the_board.servo_write(SERVO_PIN, 180)
        time.sleep(1)
        await the_board.servo_write(SERVO_PIN, 90)
    except KeyboardInterrupt:
        await the_board.shutdown()
        sys.exit(0)

    await the_board.shutdown()
    sys.exit(0)

# get the event loop
loop = asyncio.new_event_loop()
asyncio.set_event_loop(loop)

try:
    board = telemetrix_rpi_pico_w_aio.TelemetrixRpiPicoWAio(ip_address='192.168.2.102',
                                                            loop=loop)
except KeyboardInterrupt:
    sys.exit()

try:
    # start the main function
    loop.run_until_complete(move_servo(board))
    loop.run_until_complete(board.reset_board())
except KeyboardInterrupt:
    loop.run_until_complete(board.shutdown())
    sys.exit(0)

