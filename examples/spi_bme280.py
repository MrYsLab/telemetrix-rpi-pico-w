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

import time
import sys
from telemetrix_rpi_pico_w import telemetrix_rpi_pico_w

"""
Configure a BME280 for SPI operation and return a raw temperature in degrees C.

This is a vary simple demo and does not take advantage of all of the BME380 features.
It does not pretend to be accurate, but only demonstrates using the SPI api calls.
"""

# Bme280RegisterAddressConfig = 0xF5
bme_id_register = 0xd0
bme_temp_register = 0xfa
bme_reset_register = 0xe0

# data[0] = 13 - SPI report
# data[1] = SPI Port
# data[2] = BME register
# data[3 ...] = data returned


def device_callback(data):
    if data[2] == bme_id_register:
        print(f'Device ID = {data[4]}')
    # temperature is returned in 3 bytes
    # we convert
    elif data[2] == bme_temp_register:
        temp = bytes(data[3:6])
        int_val = int.from_bytes(temp, "big") / 10000
        print(f'Temp={int_val} C')


# instantiate an instance of TelemetrixRpiPicoW
board = telemetrix_rpi_pico_w.TelemetrixRpiPicoW(ip_address='192.168.102')

# set the pin mode for SPI0. Set the chip select to pin5 so that it will
# work with the Pimoroni Pico Explorer Base. Accept all other parameters as
# default
board.set_pin_mode_spi(chip_select=5)

# reset the bme280
# the data to write must be presented in the form of a list
board.spi_write_blocking([bme_reset_register])

# after resetting, we must call set pin mode again to start a new SPI.begin()
board.set_pin_mode_spi(chip_select=5)

# time.sleep(1)

# get the device ID
board.spi_read_blocking(bme_id_register, 1, spi_port=0, call_back=device_callback)

# get the raw temperature
board.spi_read_blocking(bme_temp_register, 3, spi_port=0, call_back=device_callback)

# provide time for the data to return
try:
    time.sleep(.5)
    board.shutdown()
except KeyboardInterrupt:
    board.shutdown()
