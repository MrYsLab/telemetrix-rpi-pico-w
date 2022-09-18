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


async def dummy_callback(data):
    pass

# get the event loop
loop = asyncio.new_event_loop()
asyncio.set_event_loop(loop)


# instantiate tmx_nano2040_wifi_aio
board = telemetrix_rpi_pico_w_aio.TelemetrixRpiPicoWAio(ip_address='192.168.2.102',
                                                        loop=loop)


async def set_and_get_pins(my_board):
    # set some pins to different modes
    await my_board.set_pin_mode_digital_output(4)
    await my_board.set_pin_mode_digital_input(6, callback=dummy_callback)
    # adc 0 is on gpio 26
    await my_board.set_pin_mode_analog_input(0, callback=dummy_callback)
    await my_board.set_pin_mode_digital_input_pullup(9, callback=dummy_callback)
    await my_board.set_pin_mode_neopixel(14)
    await my_board.set_pin_mode_i2c(0, 4, 5)

    print(await my_board.get_pico_pins())


try:
    # start the main function
    loop.run_until_complete(set_and_get_pins(board))
    loop.run_until_complete(board.shutdown())


except KeyboardInterrupt:
    loop.run_until_complete(board.shutdown())
    sys.exit(0)

# Create a Telemetrix instance.
board = telemetrix_rpi_pico_w_aio.TelemetrixRpiPicoWAio(ip_address='192,168.2.102')

board.shutdown()
