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
Run a motor continuously without acceleration
"""

# Create a Telemetrix instance.
board = telemetrix_rpi_pico_w.TelemetrixRpiPicoW(ip_address='192.168.2.102')


# for continuous motion, the callback is not used, but provided to meet the
# API needs.
def the_callback(data):
    date = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(data[2]))
    print(f'Run motor {data[1]} completed motion at: {date}.')


# create an accelstepper instance for a TB6600 motor driver
motor = board.set_pin_mode_stepper(interface=1, pin1=0, pin2=1)

# if you are using a 28BYJ-48 Stepper Motor with ULN2003
# comment out the line above and uncomment out the line below.
# motor = board.set_pin_mode_stepper(interface=8, pin1=8, pin2=10, pin3=9, pin4=11)


# set the max speed and speed
board.stepper_set_max_speed(motor, 900)
board.stepper_set_speed(motor, 500)

# run the motor
board.stepper_run_speed(motor)

# keep application running
while True:
    try:
        time.sleep(1)
    except KeyboardInterrupt:
        board.shutdown()
        sys.exit(0)
