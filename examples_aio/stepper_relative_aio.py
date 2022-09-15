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
import asyncio
import sys
import time

from telemetrix_rpi_pico_w_aio import telemetrix_rpi_pico_w_aio

"""
Run a motor to a relative position.

Motor used to test is a NEMA-17 size - 200 steps/rev, 12V 350mA.
And the driver is a TB6600 4A 9-42V Nema 17 Stepper Motor Driver.

The driver was connected as follows:
VCC 12 VDC
GND Power supply ground
ENA- Not connected
ENA+ Not connected
DIR- GND
DIR+ GPIO Pin 1
PUL- GND
PUL+ GPIO Pin 0
A-, A+ Coil 1 stepper motor
B-, B+ Coil 2 stepper motor
"""

# IP address assigned to the Pico W
IP_ADDRESS = '192.168.2.102'

# GPIO Pins
PULSE_PIN = 0
DIRECTION_PIN = 1

# flag to keep track of the number of times the callback
# was called. When == 2, exit program
exit_flag = 0


async def the_callback(data):
    global exit_flag

    date = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(data[2]))
    print(f'Motor {data[1]} relative  motion completed at: {date}.')
    exit_flag += 1


async def step_relative(the_board):

    global exit_flag

    # create an accelstepper instance for a TB6600 motor driver
    # if you are using a micro stepper controller board:
    # pin1 = pulse pin, pin2 = direction
    motor = await the_board.set_pin_mode_stepper(interface=1, pin1=PULSE_PIN,
                                                 pin2=DIRECTION_PIN)

    # if you are using a 28BYJ-48 Stepper Motor with ULN2003
    # comment out the line above and uncomment out the line below.
    # motor = await the_board.set_pin_mode_stepper(interface=4, pin1=5, pin2=4, pin3=14,
    # pin4=12)

    # set the max speed and acceleration
    await the_board.stepper_set_max_speed(motor, 400)
    await the_board.stepper_set_acceleration(motor, 800)

    # set the relative position in steps
    await the_board.stepper_move(motor, 2000)

    print('Running Motor')
    # run the motor
    await the_board.stepper_run(motor, completion_callback=the_callback)

    # keep application running
    while exit_flag == 0:
        try:
            await asyncio.sleep(.2)
        except KeyboardInterrupt:
            await board.shutdown()
            sys.exit(0)

    print('Reversing Direction')
    await the_board.stepper_move(motor, -2000)
    await the_board.stepper_run(motor, completion_callback=the_callback)

    while exit_flag < 2:
        try:
            await asyncio.sleep(.2)
        except KeyboardInterrupt:
            await board.shutdown()
            sys.exit(0)

    await the_board.shutdown()
    sys.exit(0)


# get the event loop
loop = asyncio.new_event_loop()
asyncio.set_event_loop(loop)

# instantiate telemetrix
board = telemetrix_rpi_pico_w_aio.TelemetrixRpiPicoWAio(ip_address=IP_ADDRESS, loop=loop)


try:
    # start the main function
    loop.run_until_complete(step_relative(board))
    loop.run_until_complete(board.shutdown())
except KeyboardInterrupt:
    loop.run_until_complete(board.shutdown())
    sys.exit(0)
    
