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


class PrivateConstants:
    """
    This class contains a set of constants for telemetrix internal use .
    """

    # commands
    # send a loop back request - for debugging communications
    LOOP_COMMAND = 0
    SET_PIN_MODE = 1  # set a pin to INPUT/OUTPUT/PWM/etc
    DIGITAL_WRITE = 2  # set a single digital pin value instead of entire port
    ANALOG_WRITE = 3
    MODIFY_REPORTING = 4
    GET_FIRMWARE_VERSION = 5
    SERVO_ATTACH = 6
    SERVO_WRITE = 7
    SERVO_DETACH = 8
    I2C_BEGIN = 9
    I2C_READ = 10
    I2C_WRITE = 11
    SONAR_NEW = 12
    DHT_NEW = 13
    STOP_ALL_REPORTS = 14
    SET_ANALOG_SCANNING_INTERVAL = 15
    ENABLE_ALL_REPORTS = 16
    RESET_DATA = 17
    SPI_INIT = 18
    SPI_WRITE_BLOCKING = 19
    SPI_READ_BLOCKING = 20
    SPI_SET_FORMAT = 21
    SPI_CS_CONTROL = 22
    ONE_WIRE_INIT = 23
    ONE_WIRE_RESET = 24
    ONE_WIRE_SELECT = 25
    ONE_WIRE_SKIP = 26
    ONE_WIRE_WRITE = 27
    ONE_WIRE_READ = 28
    ONE_WIRE_RESET_SEARCH = 29
    ONE_WIRE_SEARCH = 30
    ONE_WIRE_CRC8 = 31
    SET_PIN_MODE_STEPPER = 32
    STEPPER_MOVE_TO = 33
    STEPPER_MOVE = 34
    STEPPER_RUN = 35
    STEPPER_RUN_SPEED = 36
    STEPPER_SET_MAX_SPEED = 37
    STEPPER_SET_ACCELERATION = 38
    STEPPER_SET_SPEED = 39
    STEPPER_SET_CURRENT_POSITION = 40
    STEPPER_RUN_SPEED_TO_POSITION = 41
    STEPPER_STOP = 42
    STEPPER_DISABLE_OUTPUTS = 43
    STEPPER_ENABLE_OUTPUTS = 44
    STEPPER_SET_MINIMUM_PULSE_WIDTH = 45
    STEPPER_SET_ENABLE_PIN = 46
    STEPPER_SET_3_PINS_INVERTED = 47
    STEPPER_SET_4_PINS_INVERTED = 48
    STEPPER_IS_RUNNING = 49
    STEPPER_GET_CURRENT_POSITION = 50
    STEPPER_GET_DISTANCE_TO_GO = 51
    STEPPER_GET_TARGET_POSITION = 52
    RESET_DATA_BOARD = 53
    INIT_NEOPIXELS = 54
    SHOW_NEOPIXELS = 55
    SET_NEOPIXEL = 56
    CLEAR_NEOPIXELS = 57
    FILL_NEOPIXELS = 58
    SET_PWM_FREQ = 59
    SET_PWM_RANGE = 60
    GET_CPU_TEMPERATURE = 61

    DIGITAL_REPORT = DIGITAL_WRITE
    ANALOG_REPORT = 3
    FIRMWARE_REPORT = GET_FIRMWARE_VERSION
    SERVO_UNAVAILABLE = SERVO_ATTACH
    I2C_TOO_FEW_BYTES_RECEIVED = 8
    I2C_TOO_MANY_BYTES_RECEIVEDD = 9
    I2C_READ_REPORT = 10
    SONAR_DISTANCE = 11
    DHT_REPORT = 12
    SPI_REPORT = 13
    ONE_WIRE_REPORT = 14
    STEPPER_DISTANCE_TO_GO = 15
    STEPPER_TARGET_POSITION = 16
    STEPPER_CURRENT_POSITION = 17
    STEPPER_RUNNING_REPORT = 18
    STEPPER_RUN_COMPLETE_REPORT = 19
    CPU_TEMP_REPORT = 20

    DEBUG_PRINT = 99

    TELEMETRIX_VERSION = "1.0"

    # reporting control
    REPORTING_DISABLE_ALL = 0
    REPORTING_ANALOG_ENABLE = 1
    REPORTING_DIGITAL_ENABLE = 2
    REPORTING_ANALOG_DISABLE = 3
    REPORTING_DIGITAL_DISABLE = 4

    # Pin mode definitions

    AT_INPUT = 0
    AT_OUTPUT = 1
    AT_INPUT_PULLUP = 2
    AT_INPUT_PULL_DOWN = 3
    AT_ANALOG = 4
    AT_PWM_OUTPUT = 5
    AT_SERVO = 6
    AT_SONAR = 7
    AT_DHT = 8
    AT_I2C = 9
    AT_NEO_PIXEL = 10
    AT_SPI = 11
    AT_STEPPER = 12
    AT_ONE_WIRE = 13
    AT_MODE_NOT_SET = 255

    # flag to indicate that an i2c command does not specify a register
    I2C_NO_REGISTER = 254

    # maximum number of digital pins supported
    NUMBER_OF_DIGITAL_PINS = 100

    # maximum number of active PWM pins
    MAX_PWM_PINS_ACTIVE = 16

    # maximum number of analog pins supported
    NUMBER_OF_ANALOG_PINS = 20

    # maximum raw pwm duty cycle
    MAX_PWM_DUTY_CYCLE = 20000

    # indices to retrieve min and max duty cycles from the servo ranges dictionary
    MIN_SERVO_DUTY_CYCLE = 0
    MAX_SERVO_DUTY_CYCLE = 1

    # maximum number of sonars allowed
    MAX_SONARS = 4

    # maximum number of DHT devices allowed
    MAX_DHTS = 2

    # DHT Report subtypes
    DHT_DATA = 0
    DHT_ERROR = 1

    # NeoPixel color positions
    RED = 0
    GREEN = 1
    BLUE = 2
