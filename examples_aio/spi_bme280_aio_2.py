"""
 Copyright (c) 2022 Alan Yorinks All rights reserved.

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
import time
import sys
from telemetrix_rpi_pico_w_aio import telemetrix_rpi_pico_w_aio

"""
Configure a BME280 for SPI operation and return a raw temperature in degrees C.

This is a vary simple demo and does not take advantage of all of the BME380 features.
It does not pretend to be accurate, but only demonstrates using the SPI api calls.
"""

# mosi is tx and miso is rx

# Bme280RegisterAddressConfig = 0xF5
bme_temp_register = 0xfa
bme_reset_register = 0xe0

# data[0] = 13 - SPI report
# data[1] = SPI Port
# data[2] = BME register
# data[3 ...] = data returned


async def device_callback(data):
    # temperature is returned in 3 bytes
    # we convert
    if data[2] == bme_temp_register:
        temp = bytes(data[3:6])
        int_val = int.from_bytes(temp, "big") / 10000
        print(f'Temp={int_val} C')
    else:
        print('Unknown register')


async def read_bme_temp(my_board):
    # set the pin mode for SPI0.
    await my_board.set_pin_mode_spi(spi_port=1)

    # reset the bme280
    # the data to write must be presented in the form of a list
    await my_board.spi_write_blocking([bme_reset_register], spi_port=1)

    # after resetting, we must call set pin mode again to start a new SPI.begin()
    await my_board.set_pin_mode_spi(spi_port=1)

    # get the raw temperature
    await my_board.spi_read_blocking(bme_temp_register, 3, spi_port=1,
                               call_back=device_callback)

    await asyncio.sleep(1)


# get the event loop
loop = asyncio.new_event_loop()
asyncio.set_event_loop(loop)

# instantiate telemetrix_aio
board = telemetrix_rpi_pico_w_aio.TelemetrixRpiPicoWAio(ip_address='192.168.2.102', loop=loop)

try:
    # start the main function
    loop.run_until_complete(read_bme_temp(board))
    loop.run_until_complete(board.shutdown())
    sys.exit(0)
except KeyboardInterrupt:
    loop.run_until_complete(board.shutdown())
    sys.exit(0)

