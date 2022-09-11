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
import socket
import struct
import sys
import time
import warnings

from telemetrix_rpi_pico_w_aio.private_constants import PrivateConstants
from telemetrix_rpi_pico_w_aio.telemetrix_pico_w_aio_socket import TelemetrixAioSocket


# noinspection PyMethodMayBeStatic


class TelemetrixRpiPicoWAio:
    """
    This class exposes and implements the telemetrix API for the
    Raspberry Pi Pico W using Python asyncio for concurrency.

    It includes the public API methods as well as
    a set of private methods.

    All pin numbers are specified using PICO GPIO pin numbering.

    """

    def __init__(self, ip_address=None,
                 ip_port=31335,
                 sleep_tune=0.000001,
                 autostart=True,
                 loop=None,
                 shutdown_on_exception=True,
                 reset_on_shutdown=True):

        """

        :param ip_address: IP address assigned to the Pico W

        :param ip_port: IP Port number.

        :param sleep_tune: A tuning parameter (typically not changed by user)

        :param autostart: If you wish to call the start method within your
                          application manually, then set this to False.

        :param loop: optional user-provided event-loop

        :param shutdown_on_exception: call shutdown before raising
                                      a RunTimeError exception, or
                                      receiving a KeyboardInterrupt exception

        :param reset_on_shutdown: Reset the board upon shutdown
        """

        self.shutdown_on_exception = shutdown_on_exception

        self.reset_board_on_shutdown = reset_on_shutdown

        self.close_loop_on_shutdown = close_loop_on_shutdown

        self.autostart = autostart
        self.ip_address = ip_address
        self.ip_port = ip_port
        self.sleep_tune = sleep_tune

        # set the event loop
        if loop is None:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
        else:
            self.loop = loop

        if not ip_address:
            if self.shutdown_on_exception:
                raise RuntimeError('An IP Address MUST BE SPECIFIED')

        # check to make sure that Python interpreter is version 3.7 or greater
        python_version = sys.version_info
        if python_version[0] >= 3:
            if python_version[1] >= 7:
                pass
            else:
                raise RuntimeError("ERROR: Python 3.7 or greater is "
                                   "required for use of this program.")

        # The report_dispatch dictionary is used to process
        # incoming report messages by looking up the report message
        # and executing its associated processing method.

        self.report_dispatch = {}

        # To add a command to the command dispatch table, append here.
        self.report_dispatch.update(
            {PrivateConstants.LOOP_COMMAND: self._report_loop_data})
        self.report_dispatch.update(
            {PrivateConstants.DEBUG_PRINT: self._report_debug_data})
        self.report_dispatch.update(
            {PrivateConstants.DIGITAL_REPORT: self._digital_message})
        self.report_dispatch.update(
            {PrivateConstants.ANALOG_REPORT: self._analog_message})
        self.report_dispatch.update(
            {PrivateConstants.CPU_TEMP_REPORT: self._cpu_temp_message})
        self.report_dispatch.update(
            {PrivateConstants.FIRMWARE_REPORT: self._firmware_message})
        self.report_dispatch.update(
            {PrivateConstants.SERVO_UNAVAILABLE: self._servo_unavailable})
        self.report_dispatch.update(
            {PrivateConstants.I2C_READ_REPORT: self._i2c_read_report})
        self.report_dispatch.update(
            {
                PrivateConstants.I2C_TOO_FEW_BYTES_RECEIVED: self._i2c_too_few_bytes_received})
        self.report_dispatch.update(
            {
                PrivateConstants.I2C_TOO_MANY_BYTES_RECEIVED: self._i2c_too_many_bytes_received})
        self.report_dispatch.update(
            {PrivateConstants.SONAR_DISTANCE: self._sonar_distance_report})
        self.report_dispatch.update({PrivateConstants.DHT_REPORT: self._dht_report})
        self.report_dispatch.update({PrivateConstants.SPI_REPORT: self._spi_report})
        # self.report_dispatch.update(
        #     {PrivateConstants.ONE_WIRE_REPORT: self._onewire_report})
        self.report_dispatch.update(
            {PrivateConstants.STEPPER_DISTANCE_TO_GO:
                 self._stepper_distance_to_go_report})
        self.report_dispatch.update(
            {PrivateConstants.STEPPER_TARGET_POSITION:
                 self._stepper_target_position_report})
        self.report_dispatch.update(
            {PrivateConstants.STEPPER_CURRENT_POSITION:
                 self._stepper_current_position_report})
        self.report_dispatch.update(
            {PrivateConstants.STEPPER_RUNNING_REPORT:
                 self._stepper_is_running_report})
        self.report_dispatch.update(
            {PrivateConstants.STEPPER_RUN_COMPLETE_REPORT:
                 self._stepper_run_complete_report})
        self.report_dispatch.update(
            {PrivateConstants.STEPPER_DISTANCE_TO_GO:
                 self._stepper_distance_to_go_report})
        self.report_dispatch.update(
            {PrivateConstants.STEPPER_TARGET_POSITION:
                 self._stepper_target_position_report})

        # up to 16 pwm/servo pins may be simultaneously active
        self.pwm_active_count = 0

        # maximum pwm duty cycle - 20000
        self.maximum_pwm_duty_cycle = PrivateConstants.MAX_PWM_DUTY_CYCLE

        # dictionaries to store the callbacks for each pin
        self.analog_callbacks = {}

        self.digital_callbacks = {}

        self.cpu_temp_active = False
        self.cpu_temp_callback = None

        # there are 2 i2c ports available
        # these values help support both
        self.i2c_callback = None
        self.i2c_callback2 = None

        self.i2c_0_active = False
        self.i2c_1_active = False

        # there are 2 spi ports available
        # these values help support both
        self.spi_callback = None
        self.spi_callback2 = None

        self.spi_0_active = False
        self.spi_1_active = False

        self.spi0_chip_select = 17
        self.spi1_chip_select = 13

        # the trigger pin will be the key to retrieve
        # the callback for a specific HC-SR04
        self.sonar_callbacks = {}

        self.sonar_count = 0

        self.dht_callbacks = {}

        self.dht_count = 0

        # socket for tcp/ip communications
        self.sock = None

        # flag to indicate we are in shutdown mode
        self.shutdown_flag = False

        # debug loopback callback method
        self.loop_back_callback = None

        # flag to indicate the start of a new report
        # self.new_report_start = True

        # firmware version to be stored here
        self.firmware_version = []

        # reported pico_id
        self.reported_pico_id = []

        # Create a dictionary to store the pins in use.
        # Notice that gpio pins 23, 24 and 25 are not included
        # because the Pico does not support these GPIOs.

        # This dictionary is a list of gpio pins updated with the pin mode when a pin mode
        # is set.
        # It is created initially using a dictionary comprehension.
        self.pico_pins = {gpio_pin: PrivateConstants.AT_MODE_NOT_SET for gpio_pin in
                          range(23)}

        # skip over unavailable pins
        for pin in range(25, 29):
            self.pico_pins[pin] = PrivateConstants.AT_MODE_NOT_SET

        # create a dictionary that holds all the servo ranges
        self.servo_ranges = {gpio_pin: [1000, 2000] for gpio_pin in
                             range(23)}

        # skip over unavailable pins
        for gpio_pin in range(25, 29):
            self.servo_ranges[gpio_pin] = [1000, 2000]

        # updated when a new motor is added
        self.next_stepper_assigned = 0

        # valid list of stepper motor interface types
        self.valid_stepper_interfaces = [1, 2, 3, 4, 6, 8]

        # maximum number of steppers supported
        self.max_number_of_steppers = 4

        # number of steppers created - not to exceed the maximum
        self.number_of_steppers = 0

        # dictionary to hold stepper motor information
        self.stepper_info = {'instance': False, 'is_running': None,
                             'maximum_speed': 1, 'speed': 0, 'acceleration': 0,
                             'distance_to_go_callback': None,
                             'target_position_callback': None,
                             'current_position_callback': None,
                             'is_running_callback': None,
                             'motion_complete_callback': None,
                             'acceleration_callback': None}

        # build a list of stepper motor info items
        self.stepper_info_list = []
        # a list of dictionaries to hold stepper information
        for motor in range(self.max_number_of_steppers):
            self.stepper_info_list.append(self.stepper_info)

        self.the_reporter_thread.start()
        self.the_data_receive_thread.start()

        # neopixel data
        self.number_of_pixels = None

        self.neopixels_initiated = False

        # generic asyncio task holder
        self.the_task = None

        print(f"TelemetrixRpiPicoWAio:  Version {PrivateConstants.TELEMETRIX_VERSION}\n\n"
              f"Copyright (c) 2022 Alan Yorinks All Rights Reserved.\n")

        print('Establishing IP connection...')

        if autostart:
            self.loop.run_until_complete(self.start_aio())

    async def start_aio(self):
        """
        This method completes the instantiation of the TmxNano2040WifiAio
        class. If you set autostart to False, then your application decides
        when to complete the instantiation.
        """

        self.sock = TelemetrixAioSocket(self.ip_address, self.ip_port, self.loop)
        await self.sock.start()

        self.the_task = self.loop.create_task(self._arduino_report_dispatcher())

        # getting instance ID
        await self._get_arduino_id()

        # get telemetrix firmware version and print it
        print('\nRetrieving Telemetrix4Connect2040 firmware ID...')
        await self._get_firmware_version()
        if not self.firmware_version:
            await self._get_firmware_version()
            if not self.firmware_version:
                if self.shutdown_on_exception:
                    await self.shutdown()
                    await asyncio.sleep(.3)
                raise RuntimeError(f'Telemetrix4Connect2040 firmware version')

        else:
            print(f'Telemetrix4Connect2040 firmware version: {self.firmware_version[0]}.'
                  f'{self.firmware_version[1]}.{self.firmware_version[2]}')
        command = [PrivateConstants.ENABLE_ALL_REPORTS]
        await self.send_command(command)

    async def pwm_write(self, pin, duty_cycle=0):
        """
        Set the specified pin to the specified value.
        This is a PWM write.

        :param pin: pico GPIO pin number

        :param duty_cycle: output value - This is dependent upon
                           the PWM range. Default is 20000, but it
                           may be modified by calling pwm_range


        """
        if self.pico_pins[pin] != PrivateConstants.AT_PWM_OUTPUT \
                and self.pico_pins[pin] != PrivateConstants.AT_SERVO:
            raise RuntimeError('pwm_write: You must set the pin mode before '
                               'performing an pwm write.')

        if not (0 <= duty_cycle <= self.maximum_pwm_duty_cycle):
            raise RuntimeError('Raw PWM duty cycle out of range')

        value = duty_cycle.to_bytes(2, byteorder='big')

        command = [PrivateConstants.ANALOG_WRITE, pin, value[0], value[1]]
        await self.send_command(command)

    async def pwm_frequency(self, frequency):
        """
        Modify the pwm frequency. Valid values are in the range of 100Hz to 1MHz

        :param frequency: desired PWM write frequency
        """
        if 100 <= frequency <= 1000000000:
            freq = frequency.to_bytes(4, byteorder='big')

            command = [PrivateConstants.SET_PWM_FREQ, freq[0], freq[1], freq[2], freq[3]]
            await self.send_command(command)
        else:
            raise RuntimeError('pwm_frequency is out of range')

    async def pwm_range(self, range_pwm):
        """
        Set the duty cycle range.
        The range of values is 16 to 65535

        :param range_pwm: range value
        """

        if 16 <= range_pwm <= 65535:
            data = range_pwm.to_bytes(4, byteorder='big')
            self.maximum_pwm_duty_cycle = range_pwm

            command = [PrivateConstants.SET_PWM_RANGE, data[0], data[1],
                       data[2], data[3]]
            await self.send_command(command)
        else:
            raise RuntimeError('pwm_range is out of range')

    async def digital_write(self, pin, value):
        """
        Set the specified pin to the specified value.

        :param pin: pico GPIO pin number

        :param value: pin value (1 or 0)

        """
        if self.pico_pins[pin] != PrivateConstants.AT_OUTPUT:
            raise RuntimeError('digital_write: You must set the pin mode before '
                               'performing a digital write.')
        command = [PrivateConstants.DIGITAL_WRITE, pin, value]
        await self.send_command(command)

    async def disable_all_reporting(self):
        """
        Disable reporting for all digital and analog input pins
        """
        command = [PrivateConstants.MODIFY_REPORTING,
                   PrivateConstants.REPORTING_DISABLE_ALL, 0]
        await self.send_command(command)

    async def disable_analog_reporting(self, pin):
        """
        Disables analog reporting for a single analog pin.

        :param pin: Analog pin number. For example for ADC0, the number is 0.

        """
        command = [PrivateConstants.MODIFY_REPORTING,
                   PrivateConstants.REPORTING_ANALOG_DISABLE, pin]
        await self.send_command(command)

    async def disable_digital_reporting(self, pin):
        """
        Disables digital reporting for a single digital input.

        :param pin: GPIO Pin number.

        """
        command = [PrivateConstants.MODIFY_REPORTING,
                   PrivateConstants.REPORTING_DIGITAL_DISABLE, pin]
        await self.send_command(command)

    async def enable_analog_reporting(self, pin):
        """
        Enables analog reporting for the specified pin.

        :param pin: Analog pin number. For example for ADC0, the number is 0.


        """
        command = [PrivateConstants.MODIFY_REPORTING,
                   PrivateConstants.REPORTING_ANALOG_ENABLE, pin]
        await self.send_command(command)

    async def enable_digital_reporting(self, pin):
        """
        Enable reporting on the specified digital pin.

        :param pin: GPIO Pin number.
        """

        command = [PrivateConstants.MODIFY_REPORTING,
                   PrivateConstants.REPORTING_DIGITAL_ENABLE, pin]
        await self.send_command(command)

    async def _get_firmware_version(self):
        """
        This method retrieves the
        pico-telemetrix firmware version

        """
        command = [PrivateConstants.GET_FIRMWARE_VERSION]
        await self.send_command(command)
        # provide time for the reply
        time.sleep(.5)

    async def i2c_read(self, address, register, number_of_bytes,
                       callback=None, i2c_port=0, send_stop=True):
        """
        Read the specified number of bytes from the specified register for
        the i2c device.


        :param address: i2c device address

        :param register: i2c register (or None if no register selection is needed)

        :param number_of_bytes: number of bytes to be read

        :param callback: Required callback function to report i2c data as a
                   result of read command

       :param i2c_port: 0 = port 0, 1 = port 1

       :param send_stop: If False, master retains control of the bus at the end of the
                       transfer (no Stop is issued), and the next transfer will
                       begin with a Restart rather than a Start.


        callback returns a data list:
        [I2C_READ_REPORT, i2c_port, i2c_device_address, count of data bytes,
        data bytes, time-stamp]

        I2C_READ_REPORT = 10

        """

        if not callback:
            if self.shutdown_on_exception:
                await self.shutdown()
            raise RuntimeError('I2C Read: A callback function must be specified.')

        # i2c_port = 0 for port 0
        if i2c_port == 0:
            if not self.i2c_0_active:
                if self.shutdown_on_exception:
                    await self.shutdown()
                raise RuntimeError(
                    'I2C Read: set_pin_mode_i2c never called for i2c port 0.')
            else:
                self.i2c_callback = callback

        else:
            if i2c_port == 1:
                if not self.i2c_1_active:
                    if self.shutdown_on_exception:
                        await self.shutdown()
                    raise RuntimeError(
                        'I2C READ: set_pin_mode_i2c never called for i2c port 1.')
                else:
                    self.i2c_callback2 = callback

        command = [PrivateConstants.I2C_READ, i2c_port, address, register,
                   number_of_bytes, send_stop]

        # no register specified
        if not register:
            command[3] = PrivateConstants.I2C_NO_REGISTER

        await self.send_command(command)

    async def i2c_write(self, address, args, i2c_port=0):
        """
        Write data to an i2c device.

        :param address: i2c device address

        :param args: A variable number of bytes to be sent to the device
                     passed in as a list.
                     NOTE: THIS MUST BE IN THE FORM OF A LIST.

        :param i2c_port: 0= port 0, 1 = port 1

        """
        if not i2c_port:
            if not self.i2c_0_active:
                if self.shutdown_on_exception:
                    await self.shutdown()
                raise RuntimeError(
                    'I2C Write: set_pin_mode i2c never called for i2c port 0.')

        elif i2c_port:
            if not self.i2c_1_active:
                if self.shutdown_on_exception:
                    await self.shutdown()
                raise RuntimeError(
                    'I2C Write: set_pin_mode i2c never called for i2c port 2.')

        if type(args) != list:
            raise RuntimeError('args must be in the form of a list')

        command = [PrivateConstants.I2C_WRITE, i2c_port, address, len(args)]

        for item in args:
            command.append(item)

        await self.send_command(command)

    async def neo_pixel_set_value(self, pixel_number, r=0, g=0, b=0, auto_show=False):
        """
        Set the selected pixel in the pixel array on the Pico to
        the value provided.

        :param pixel_number: pixel number

        :param r: red value 0-255

        :param g: green value 0-255

        :param b: blue value 0-255

        :param auto_show: call show automatically

        """
        if not self.neopixels_initiated:
            raise RuntimeError('You must call set_pin_mode_neopixel first')

        if pixel_number > self.number_of_pixels:
            raise RuntimeError('Pixel number is out of legal range')

        if r and g and b not in range(256):
            raise RuntimeError('Pixel value must be in the range of 0-255')

        command = [PrivateConstants.SET_NEOPIXEL, pixel_number, r, g, b, auto_show]
        await self.send_command(command)

        if auto_show:
            await self.shutdown()

    async def neopixel_clear(self, auto_show=True):
        """
        Clear all pixels

        :param auto_show: call show automatically

        """
        if not self.neopixels_initiated:
            raise RuntimeError('You must call set_pin_mode_neopixel first')
        command = [PrivateConstants.CLEAR_NEOPIXELS, auto_show]
        await self.send_command(command)
        if auto_show:
            await self.shutdown()

    async def neopixel_fill(self, r=0, g=0, b=0, auto_show=True):
        """
        Fill all pixels with specified value

        :param r: 0-255

        :param g: 0-255

        :param b: 0-255

        :param auto_show: call show automatically
        """
        if not self.neopixels_initiated:
            raise RuntimeError('You must call set_pin_mode_neopixel first')
        if r and g and b not in range(256):
            raise RuntimeError('Pixel value must be in the range of 0-255')
        command = [PrivateConstants.FILL_NEOPIXELS, r, g, b, auto_show]
        await self.send_command(command)

        if auto_show:
            await self.shutdown()

    async def neopixel_show(self):
        """
        Write the NeoPixel buffer stored in the Pico to the NeoPixel strip.

        """
        if not self.neopixels_initiated:
            raise RuntimeError('You must call set_pin_mode_neopixel first')
        command = [PrivateConstants.SHOW_NEOPIXELS]
        await self.send_command(command)

    async def loop_back(self, start_character, callback=None):
        """
        This is a debugging method to send a character to the
        pico device, and have the device loop it back.

        :param start_character: The character to loop back. It should be
                                an integer.

        :param callback: Looped back character will appear in the callback method

        """
        command = [PrivateConstants.LOOP_COMMAND, ord(start_character)]
        self.loop_back_callback = callback
        await self.send_command(command)

    async def set_pin_mode_analog_input(self, adc_number, differential=0, callback=None):
        """
        Set a pin as an analog input.

        :param adc_number: ADC Number 0-4

                           ADC numbers are mapped as following:
                           ADC0 = GPIO 26 (Physical Pin 31)
                           ADC1 = GPIO 27 (Physical Pin 32)
                           ADC2 = GPIO 28 (Physical Pin 34)

                           Internal Mapping
                           ADC3 = GPIO 29 (Physical Pin 35) ADC Reference Voltage

                           NOTE: This is different from telemetrix-rpi-pico
                           To get cpu temperature, call get_cpu_temperature.

        :param differential: difference in previous to current value before
                             report will be generated

        :param callback: callback function


        callback returns a data list:

        [ANALOG_REPORT, pin_number, pin_value, raw_time_stamp]

        The ANALOG_REPORT  = 3

        """
        # make sure adc number is in range
        if not 0 <= adc_number < 4:
            raise RuntimeError('Invalid ADC Number')
        await self._set_pin_mode(adc_number, PrivateConstants.AT_ANALOG, differential,
                           callback=callback)

    async def get_cpu_temperature(self, threshold=1.0, polling_interval=1000,
                                  callback=None):
        """
        Request the CPU temperature. This will continuously monitor the temperature
        and report it back in degrees celsius. Call only once, unless you wish to
        modify the polling interval.

        :param threshold:    The threshold value is used to determine when a
        temperature report is generated. The current temperature is compared to
        plus and minus the threshold value and if the value is exceeded, a report is
        generated. To receive continuous reports, set the threshold to 0. A maximum of
        5.0 degrees is allowed.

        :param polling_interval: number of milliseconds between temperature reads.
                                 Maximum of 60 seconds (6000 ms.)

        :param callback: callback function

        callback returns a list:
        [CPU_TEMPERATURE_REPORT, degrees_celsius, raw_time_stamp]

        CPU_TEMPERATURE_REPORT = 20
        """

        if not callback:
            raise RuntimeError('get_cpu_temperature: you must specify a callback')
        # convert the floating point threshold to bytes

        if 0.0 <= threshold < 30.0:
            if 0 <= polling_interval < 60000:
                thresh_list = list(struct.pack("f", threshold))
                polling_list = polling_interval.to_bytes(2, byteorder='big')
                self.cpu_temp_callback = callback

                self.cpu_temp_active = True

                command = [PrivateConstants.GET_CPU_TEMPERATURE, thresh_list[0],
                           thresh_list[1],
                           thresh_list[2], thresh_list[3], polling_list[0],
                           polling_list[1]]

                await self.send_command(command)
            else:
                raise RuntimeError('get_cpu_temperature: polling interval out of range.')
        else:
            raise RuntimeError('get_cpu_temperature: threshold out of range.')

    async def set_pin_mode_digital_input(self, pin_number, callback=None):
        """
        Set a pin as a digital input.

        :param pin_number: pico GPIO pin number

        :param callback: callback function


        callback returns a data list:

        [DIGITAL_REPORT, pin_number, pin_value, raw_time_stamp]

        DIGITAL_REPORT = 2

        """
        await self._set_pin_mode(pin_number, PrivateConstants.AT_INPUT, callback=callback)

    async def set_pin_mode_digital_input_pullup(self, pin_number, callback=None):
        """
        Set a pin as a digital input with pullup enabled.

        :param pin_number: pico GPIO pin number

        :param callback: callback function


        callback returns a data list:

        [DIGITAL_REPORT, pin_number, pin_value, raw_time_stamp]

        The DIGITAL_REPORT = 2

        """
        await self._set_pin_mode(pin_number, PrivateConstants.AT_INPUT_PULLUP,
                           callback=callback)

    async def set_pin_mode_digital_input_pull_down(self, pin_number, callback=None):
        """
        Set a pin as a digital input with pull down enabled.

        :param pin_number: pico GPIO pin number

        :param callback: callback function


        callback returns a data list:

        [DIGITAL_REPORT, pin_number, pin_value, raw_time_stamp]

        DIGITAL_REPORT= 2

        """
        await self._set_pin_mode(pin_number, PrivateConstants.AT_INPUT_PULL_DOWN,
                           callback=callback)

    async def set_pin_mode_digital_output(self, pin_number):
        """
        Set a pin as a digital output pin.

        :param pin_number: pico GPIO pin number
        """

        await self._set_pin_mode(pin_number, PrivateConstants.AT_OUTPUT)

    async def set_pin_mode_neopixel(self, pin_number=28, num_pixels=8,
                                    fill_r=0, fill_g=0, fill_b=0):
        """
        Initialize the pico for NeoPixel control. Fill with rgb values specified.

        Default: Set all the pixels to off.

        :param pin_number: neopixel GPIO control pin

        :param num_pixels: number of pixels in the strip

        :param fill_r: initial red fill value 0-255

        :param fill_g: initial green fill value 0-255

        :param fill_b: initial blue fill value 0-255


        """
        if fill_r or fill_g or fill_g not in range(256):
            raise RuntimeError('Pixel value must be in the range of 0-255')

        self.number_of_pixels = num_pixels

        command = [PrivateConstants.INIT_NEOPIXELS, pin_number,
                   self.number_of_pixels, fill_r, fill_g, fill_b]

        await self.send_command(command)

        self.pico_pins[pin_number] = PrivateConstants.AT_NEO_PIXEL

        self.neopixels_initiated = True

    async def set_pin_mode_pwm_output(self, pin_number):
        """
        Enable a pin as a PWM pin. Maximum number of PWMs is 16.

        Note: There are up to 16 pins that can be assigned as
        PWM. Servo pins share the 16 PWM pins.

        :param pin_number: pico GPIO pin number

        """

        if pin_number in self.pico_pins:
            self.pico_pins[pin_number] = PrivateConstants.AT_PWM_OUTPUT
            if self.pwm_active_count >= 15:
                raise RuntimeError(
                    'pwm or servo set mode: number of active PWM pins is at maximum')

            self.pwm_active_count += 1

            await self._set_pin_mode(pin_number, PrivateConstants.AT_PWM_OUTPUT)
        else:
            raise RuntimeError('Gpio Pin Number is invalid')

    async def set_pin_mode_i2c(self, i2c_port=0, sda_gpio=None, scl_gpio=None):
        """
        Establish the standard pico i2c pins for i2c utilization.

        SDA pins: port 0 = 4   port 1 = 26
        SCL pins: port 0 = 5   port 1 = 27

        :param i2c_port: 0 = i2c0, 1 = i2c1

        :param sda_gpio: gpio pin assigned to SDA

        :param scl_gpio: gpio pin assigned to SCL


        NOTES:
               1. THIS METHOD MUST BE CALLED BEFORE ANY I2C REQUEST IS MADE <br>
               2. Callbacks are set within the individual i2c read methods of this
              API.

              See i2c_read, and i2c_write

        """

        # determine if the i2c port is specified correctly
        if i2c_port not in [0, 1]:
            raise RuntimeError('i2c port must be either a 0 or 1')
        if sda_gpio or scl_gpio:
            if i2c_port == 0:
                warnings.warn('SDA = 4, and SCL = 5. Ignoring pins specified')
            else:
                warnings.warn('SDA = 26, and SCL = 27. Ignoring pins specified')

        # test for i2c port 0
        if not i2c_port:
            self.i2c_0_active = True
            sda_gpio = 4
            scl_gpio = 5
        # port 1
        else:
            self.i2c_1_active = True
            sda_gpio = 26
            scl_gpio = 27

        self.pico_pins[sda_gpio] = self.pico_pins[scl_gpio] = PrivateConstants.AT_I2C

        command = [PrivateConstants.I2C_BEGIN, i2c_port]
        await self.send_command(command)

    async def set_pin_mode_dht(self, pin, callback=None):
        """

      :param pin: connection pin

      :param callback: callback function

      callback returns a data list:

    DHT REPORT, DHT_DATA=1, PIN, Humidity,  Temperature (c),Time]

    DHT_REPORT =  12

        """

        if not callback:
            if self.shutdown_on_exception:
                await self.shutdown()
            raise RuntimeError('set_pin_mode_dht: A Callback must be specified')

        if self.dht_count < PrivateConstants.MAX_DHTS:
            self.dht_callbacks[pin] = callback
            self.dht_count += 1
            self.pico_pins[pin] = PrivateConstants.AT_DHT
            command = [PrivateConstants.DHT_NEW, pin]
            await self.send_command(command)
        else:
            if self.shutdown_on_exception:
                await self.shutdown()
            raise RuntimeError(
                f'Maximum Number Of DHTs Exceeded - set_pin_mode_dht fails for pin {pin}')

    async def set_pin_mode_servo(self, pin_number, min_pulse=1000, max_pulse=2000):
        """

        Attach a pin to a servo motor

        There are 16 PWM pins shared between the Servo and PWM Output modes.

        :param pin_number: pin

        :param min_pulse: minimum pulse width in microseconds

        :param max_pulse: maximum pulse width in microseconds

        """

        if pin_number in self.pico_pins:
            self.pico_pins[pin_number] = PrivateConstants.AT_PWM_OUTPUT
            if self.pwm_active_count >= 15:
                raise RuntimeError(
                    'pwm or servo set mode: number of active PWM pins is at maximum')

            self.pwm_active_count += 1

        await self._set_pin_mode(pin_number, PrivateConstants.AT_SERVO, min_pulse, max_pulse)
        self.pico_pins[pin_number] = PrivateConstants.AT_SERVO

    async def set_pin_mode_spi(self, spi_port=0, chip_select=None, speed_maximum=500000,
                               data_order=1, data_mode=0):

        """
        Specify the SPI port. The SPI port is configured as a "master".
        Optionally specify the chip select pin.

        This command also specifies the SPISettings for the selected SPI port.

        This method sets the SPISettings structure based on parameter values
        provided.

        SPI support is provided as an abstraction. It is intended that
        only a single device is connected to an SPI port.

        :param spi_port: 0 = spi0, 1 = spi1

        :param chip_select:  spi0 = 17  spi1 = 13 or select another pin

        :param speed_maximum: The maximum speed of communication. Maximum is 50000000

        :param data_order: 1=MSBFIRST,  0=LSBFIRST,

        :param data_mode: 0=MODE0, 1=MODE1, 2=MODE2 3=MODE3

        miso: SPI data receive pin   spi0 = 16  spi1 = 12

        mosi: SPI data transmit pin  spi0 = 19  spi1 = 15

        clock_pin: clock pin         spi0 = 18  spi1 = 14


        command message: [command, spi, chip_select, speed, data_order, data_mode]
        """
        # determine if the spi port is specified correctly
        if spi_port not in [0, 1]:
            if self.shutdown_on_exception:
                await self.shutdown()
            raise RuntimeError('spi port must be either a 0 or 1')

        if not (0 < speed_maximum < 50000000):
            if self.shutdown_on_exception:
                await self.shutdown()
            raise RuntimeError('spi port speed maximum is out of range')

        if data_order not in [0, 1]:
            if self.shutdown_on_exception:
                await self.shutdown()
            raise RuntimeError('spi data order must be 0 or 1')

        if data_mode not in [0, 1, 2, 3]:
            if self.shutdown_on_exception:
                await self.shutdown()
            raise RuntimeError('spi data data mode must be 0, 1, 3 or 3')

        if spi_port == 0:
            self.spi_0_active = True
            miso = 16
            mosi = 19
            if chip_select:
                self.spi0_chip_select = chip_select
            else:
                chip_select = 17
            clock_pin = 18

        else:
            self.spi_1_active = True
            miso = 12
            mosi = 15
            if chip_select:
                self.spi1_chip_select = chip_select
            else:
                chip_select = 13
            clock_pin = 14

        freq_bytes = speed_maximum.to_bytes(4, byteorder='big')

        self.pico_pins[mosi] = PrivateConstants.AT_SPI
        self.pico_pins[miso] = PrivateConstants.AT_SPI
        self.pico_pins[clock_pin] = PrivateConstants.AT_SPI
        self.pico_pins[chip_select] = PrivateConstants.AT_SPI

        command = [PrivateConstants.SPI_INIT, spi_port, chip_select, freq_bytes[0],
                   freq_bytes[1], freq_bytes[2], freq_bytes[3], data_order, data_mode]
        await self.send_command(command)

    async def set_pin_mode_stepper(self, interface=1, pin1=2, pin2=3, pin3=4,
                                   pin4=5, enable=True):
        """
        Stepper motor support is implemented as a proxy for
        the AccelStepper library.

        https://github.com/waspinator/AccelStepper

        Instantiate a stepper motor.

        Initialize the interface and pins for a stepper motor.

        :param interface: Motor Interface Type:

                1 = Stepper Driver, 2 driver pins required

                2 = FULL2WIRE  2 wire stepper, 2 motor pins required

                3 = FULL3WIRE 3 wire stepper, such as HDD spindle,
                    3 motor pins required

                4 = FULL4WIRE, 4 wire full stepper, 4 motor pins
                    required

                6 = HALF3WIRE, 3 wire half stepper, such as HDD spindle,
                    3 motor pins required

                8 = HALF4WIRE, 4 wire half stepper, 4 motor pins required

        :param pin1: Pico digital pin number for motor pin 1

        :param pin2: Pico digital pin number for motor pin 2

        :param pin3: Pico digital pin number for motor pin 3

        :param pin4: Pico digital pin number for motor pin 4

        :param enable: If this is true, the output pins are enabled at construction time.

        :return: Motor Reference number
        """

        if self.number_of_steppers == self.max_number_of_steppers:
            if self.shutdown_on_exception:
                await self.shutdown()
            raise RuntimeError('Maximum number of steppers has already been assigned')

        if interface not in self.valid_stepper_interfaces:
            if self.shutdown_on_exception:
                await self.shutdown()
            raise RuntimeError('Invalid stepper interface')

        self.number_of_steppers += 1

        motor_id = self.next_stepper_assigned
        self.next_stepper_assigned += 1
        self.stepper_info_list[motor_id]['instance'] = True

        # build message and send message to server
        command = [PrivateConstants.SET_PIN_MODE_STEPPER, motor_id, interface, pin1,
                   pin2, pin3, pin4, enable]
        await self.send_command(command)

        # return motor id
        return motor_id

    async def servo_write(self, pin_number, value):
        """
        Write the value to the specified servo

        :param pin_number: GPIO pin number

        :param value: value between 0 and 180

        """

        if self.pico_pins[pin_number] != PrivateConstants.AT_SERVO:
            raise RuntimeError('You must call set_pin_mode_servo before trying to '
                               'write a value to a servo or servo was detached.')

        # get the min and max for the servo and calculate the duty-cycle
        min_duty = self.servo_ranges[pin_number][PrivateConstants.MIN_SERVO_DUTY_CYCLE]
        max_duty = self.servo_ranges[pin_number][PrivateConstants.MAX_SERVO_DUTY_CYCLE]

        mm = min_duty.to_bytes(2, byteorder='big')
        mx = max_duty.to_bytes(2, byteorder='big')

        command = [PrivateConstants.SERVO_WRITE, pin_number, value, mm[0], mm[1],
                   mx[0], mx[1]]
        await self.send_command(command)

    async def set_pin_mode_sonar(self, trigger_pin, echo_pin, callback=None):
        """
        :param trigger_pin:  Sensor trigger gpio pin

        :param echo_pin: Sensor echo gpio pin

        :param callback: callback

       callback returns a data list:

       [ SONAR_DISTANCE, trigger_pin, distance_value, time_stamp]

       SONAR_DISTANCE =  11

        """

        if not callback:
            if self.shutdown_on_exception:
                await self.shutdown()
            raise RuntimeError('set_pin_mode_sonar: A Callback must be specified')

        if self.sonar_count < PrivateConstants.MAX_SONARS:
            self.sonar_callbacks[trigger_pin] = callback
            self.sonar_count += 1
            self.pico_pins[trigger_pin] = self.pico_pins[echo_pin] = \
                PrivateConstants.AT_SONAR

            command = [PrivateConstants.SONAR_NEW, trigger_pin, echo_pin]
            await self.send_command(command)
        else:
            if self.shutdown_on_exception:
                await self.shutdown()
            raise RuntimeError('Maximum number of supported sonar devices exceeded.')

    async def spi_read_blocking(self, register, number_of_bytes, spi_port=0,
                                call_back=None):
        """
        Read the specified number of bytes from the specified SPI port and
        call the callback function with the reported data.

        On the server side, this command incorporates SPI.beginTransaction
        and chip select control.

        :param register: Register to be selected

        :param number_of_bytes: Number of bytes to read

        :param spi_port: SPI port 0 or 1

        :param call_back: Required callback function to report spi data as a
                   result of read command


        callback returns a data list:
        [SPI_READ_REPORT, spi_port, count of data bytes, data bytes, time-stamp]

        SPI_READ_REPORT = 13

        """
        if spi_port == 0:
            if not self.spi_0_active:
                if self.shutdown_on_exception:
                    await self.shutdown()
                raise RuntimeError(
                    'spi_read_blocking: set_pin_mode_spi never called for spi port 0.')

        elif spi_port == 1:
            if not self.spi_1_active:
                if self.shutdown_on_exception:
                    await self.shutdown()
                raise RuntimeError(
                    'spi_read_blocking: set_pin_mode_spi never called for spi port 1.')

        if not call_back:
            if self.shutdown_on_exception:
                await self.shutdown()
            raise RuntimeError('spi_read_blocking: A Callback must be specified')
        if spi_port == 0:
            self.spi_callback = call_back
        else:
            self.spi_callback2 = call_back

        command = [PrivateConstants.SPI_READ_BLOCKING, spi_port, register,
                   number_of_bytes]
        await self.send_command(command)

    async def spi_write_blocking(self, bytes_to_write, spi_port=0):
        """
        Write a list of bytes to the SPI device.

        On the server side, this command incorporates SPI.beginTransaction
        and chip select control.

        :param bytes_to_write: A list of bytes to write. This must be in the form of a
        list.

        :param spi_port: SPI port 0 or 1

        """
        if spi_port == 0:
            if not self.spi_0_active:
                if self.shutdown_on_exception:
                    await self.shutdown()
                raise RuntimeError(
                    'spi_write_blocking: set_pin_mode_spi never called for spi port 0.')

        elif spi_port:
            if not self.spi_1_active:
                if self.shutdown_on_exception:
                    await self.shutdown()
                raise RuntimeError(
                    'spi_write_blocking: set_pin_mode_spi never called for spi port 1.')
        command = [PrivateConstants.SPI_WRITE_BLOCKING, spi_port,
                   len(bytes_to_write)]

        for data in bytes_to_write:
            command.append(data)

        await self.send_command(command)

    async def get_pico_pins(self):
        """
        This method returns the pico_pins dictionary

        Pin Modes MAP:

            DIGITAL_INPUT = 0

            DIGITAL_OUTPUT = 1

            PWM_OUTPUT = 2

            DIGITAL_INPUT_PULLUP = 3

            DIGITAL_INPUT_PULL_DOWN = 4

            ANALOG_INPUT = 5

            SERVO = 6

            SONAR = 7

            DHT = 8

            I2C = 9

            NEO_PIXEL = 10

            AT_MODE_NOT_SET = 255

        :return: pico_pins
        """
        return self.pico_pins

    async def stepper_move_to(self, motor_id, position):
        """
        Set an absolution target position. If position is positive, the movement is
        clockwise, else it is counter-clockwise.

        The run() function (below) will try to move the motor (at most one step per call)
        from the current position to the target position set by the most
        recent call to this function. Caution: moveTo() also recalculates the
        speed for the next step.
        If you are trying to use constant speed movements, you should call setSpeed()
        after calling moveTo().

        :param motor_id: motor id: 0 - 3

        :param position: target position. Maximum value is 32 bits.
        """
        if position < 0:
            polarity = 1
        else:
            polarity = 0
        position = abs(position)
        if not self.stepper_info_list[motor_id]['instance']:
            if self.shutdown_on_exception:
                await self.shutdown()
            raise RuntimeError('stepper_move_to: Invalid motor_id.')

        position_bytes = list(position.to_bytes(4, 'big', signed=True))

        command = [PrivateConstants.STEPPER_MOVE_TO, motor_id]
        for value in position_bytes:
            command.append(value)
        command.append(polarity)
        await self.send_command(command)

    async def stepper_move(self, motor_id, relative_position):
        """
        Set the target position relative to the current position.

        :param motor_id: motor id: 0 - 3

        :param relative_position: The desired position relative to the current
                                  position. Negative is anticlockwise from
                                  the current position. Maximum value is 32 bits.
        """
        if relative_position < 0:
            polarity = 1
        else:
            polarity = 0

        relative_position = abs(relative_position)
        if not self.stepper_info_list[motor_id]['instance']:
            if self.shutdown_on_exception:
                await self.shutdown()
            raise RuntimeError('stepper_move: Invalid motor_id.')

        position_bytes = list(relative_position.to_bytes(4, 'big', signed=True))

        command = [PrivateConstants.STEPPER_MOVE, motor_id]
        for value in position_bytes:
            command.append(value)
        command.append(polarity)
        await self.send_command(command)

    async def stepper_run(self, motor_id, completion_callback=None):
        """
        This method steps the selected motor based on the current speed.

        Once called, the server will continuously attempt to step the motor.

        :param motor_id: 0 - 3

        :param completion_callback: call back function to receive motion complete
                                    notification

        callback returns a data list:

        [report_type, motor_id, raw_time_stamp]

        The report_type = 19
        """
        if not completion_callback:
            if self.shutdown_on_exception:
                await self.shutdown()
            raise RuntimeError('stepper_run: A motion complete callback must be '
                               'specified.')

        if not self.stepper_info_list[motor_id]['instance']:
            if self.shutdown_on_exception:
                await self.shutdown()
            raise RuntimeError('stepper_run: Invalid motor_id.')

        self.stepper_info_list[motor_id]['motion_complete_callback'] = completion_callback
        command = [PrivateConstants.STEPPER_RUN, motor_id]
        await self.send_command(command)

    async def stepper_run_speed(self, motor_id):
        """
        This method steps the selected motor based at a constant speed as set by the most
        recent call to stepper_set_max_speed(). The motor will run continuously.

        Once called, the server will continuously attempt to step the motor.

        :param motor_id: 0 - 3

        """
        if not self.stepper_info_list[motor_id]['instance']:
            if self.shutdown_on_exception:
                await self.shutdown()
            raise RuntimeError('stepper_run_speed: Invalid motor_id.')

        command = [PrivateConstants.STEPPER_RUN_SPEED, motor_id]
        await self.send_command(command)

    async def stepper_set_max_speed(self, motor_id, max_speed):
        """
        Sets the maximum permitted speed. The stepper_run() function will accelerate
        up to the speed set by this function.

        Caution: the maximum speed achievable depends on your processor and clock speed.
        The default maxSpeed is 1 step per second.

         Caution: Speeds that exceed the maximum speed supported by the processor may
                  result in non-linear accelerations and decelerations.

        :param motor_id: 0 - 3

        :param max_speed: 1 - 1000
        """

        if not self.stepper_info_list[motor_id]['instance']:
            if self.shutdown_on_exception:
                await self.shutdown()
            raise RuntimeError('stepper_set_max_speed: Invalid motor_id.')

        if not 1 < max_speed <= 1000:
            if self.shutdown_on_exception:
                await self.shutdown()
            raise RuntimeError('stepper_set_max_speed: Speed range is 1 - 1000.')

        self.stepper_info_list[motor_id]['max_speed'] = max_speed
        max_speed_msb = (max_speed & 0xff00) >> 8
        max_speed_lsb = max_speed & 0xff

        command = [PrivateConstants.STEPPER_SET_MAX_SPEED, motor_id, max_speed_msb,
                   max_speed_lsb]
        await self.send_command(command)

    async def stepper_get_max_speed(self, motor_id):
        """
        Returns the maximum speed configured for this stepper
        that was previously set by stepper_set_max_speed()

        Value is stored in the client, so no callback is required.

        :param motor_id: 0 - 3

        :return: The currently configured maximum speed.
        """
        if not self.stepper_info_list[motor_id]['instance']:
            if self.shutdown_on_exception:
                await self.shutdown()
            raise RuntimeError('stepper_max_speed: Invalid motor_id.')

        return self.stepper_info_list[motor_id]['max_speed']

    async def stepper_set_acceleration(self, motor_id, acceleration):
        """
        Sets the acceleration/deceleration rate.

        :param motor_id: 0 - 3

        :param acceleration: The desired acceleration in steps
                             per second. Must be > 0.0. This is an
                             expensive call since it requires a square
                             root to be calculated on the server.
                             Don't call more often than needed.

        """
        if not self.stepper_info_list[motor_id]['instance']:
            if self.shutdown_on_exception:
                await self.shutdown()
            raise RuntimeError('stepper_set_acceleration: Invalid motor_id.')

        if not 1 < acceleration <= 1000:
            if self.shutdown_on_exception:
                await self.shutdown()
            raise RuntimeError('stepper_set_acceleration: Acceleration range is 1 - '
                               '1000.')

        self.stepper_info_list[motor_id]['acceleration'] = acceleration

        max_accel_msb = acceleration >> 8
        max_accel_lsb = acceleration & 0xff

        command = [PrivateConstants.STEPPER_SET_ACCELERATION, motor_id, max_accel_msb,
                   max_accel_lsb]
        await self.send_command(command)

    async def stepper_set_speed(self, motor_id, speed):
        """
        Sets the desired constant speed for use with stepper_run_speed().

        :param motor_id: 0 - 3

        :param speed: 0 - 1000 The desired constant speed in steps per
                      second. Positive is clockwise. Speeds of more than 1000 steps per
                      second are unreliable. Speed accuracy depends on the board
                      crystal. Jitter depends on how frequently you call the
                      stepper_run_speed() method.
                      The speed will be limited by the current value of
                      stepper_set_max_speed().
        """
        if not self.stepper_info_list[motor_id]['instance']:
            if self.shutdown_on_exception:
                await self.shutdown()
            raise RuntimeError('stepper_set_speed: Invalid motor_id.')

        if not 0 < speed <= 1000:
            if self.shutdown_on_exception:
                await self.shutdown()
            raise RuntimeError('stepper_set_speed: Speed range is 0 - '
                               '1000.')

        self.stepper_info_list[motor_id]['speed'] = speed

        speed_msb = speed >> 8
        speed_lsb = speed & 0xff

        command = [PrivateConstants.STEPPER_SET_SPEED, motor_id, speed_msb, speed_lsb]
        await self.send_command(command)

    async def stepper_get_speed(self, motor_id):
        """
        Returns the  most recently set speed.
        that was previously set by stepper_set_speed();

        Value is stored in the client, so no callback is required.

        :param motor_id:  0 - 3

        """
        if not self.stepper_info_list[motor_id]['instance']:
            if self.shutdown_on_exception:
                await self.shutdown()
            raise RuntimeError('stepper_get_speed: Invalid motor_id.')

        return self.stepper_info_list[motor_id]['speed']

    async def stepper_get_distance_to_go(self, motor_id, distance_to_go_callback):
        """
        Request the distance from the current position to the target position
        from the server.

        :param motor_id: 0 - 3

        :param distance_to_go_callback: required callback function to receive report

        :return: The distance to go is returned via the callback as a list:

        [REPORT_TYPE=15, motor_id, distance in steps, time_stamp]

        A positive distance is clockwise from the current position.

        """
        if not distance_to_go_callback:
            if self.shutdown_on_exception:
                await self.shutdown()
            raise RuntimeError(
                'stepper_get_distance_to_go Read: A callback function must be specified.')

        if not self.stepper_info_list[motor_id]['instance']:
            if self.shutdown_on_exception:
                await self.shutdown()
            raise RuntimeError('stepper_get_distance_to_go: Invalid motor_id.')
        self.stepper_info_list[motor_id][
            'distance_to_go_callback'] = distance_to_go_callback
        command = [PrivateConstants.STEPPER_GET_DISTANCE_TO_GO, motor_id]
        await self.send_command(command)

    async def stepper_get_target_position(self, motor_id, target_callback):
        """
        Request the most recently set target position from the server.

        :param motor_id: 0 - 3

        :param target_callback: required callback function to receive report

        :return: The distance to go is returned via the callback as a list:

        [REPORT_TYPE=16, motor_id, target position in steps, time_stamp]

        Positive is clockwise from the 0 position.

        """
        if not target_callback:
            if self.shutdown_on_exception:
                await self.shutdown()
            raise RuntimeError(
                'stepper_get_target_position Read: A callback function must be specified.')

        if not self.stepper_info_list[motor_id]['instance']:
            if self.shutdown_on_exception:
                await self.shutdown()
            raise RuntimeError('stepper_get_target_position: Invalid motor_id.')

        self.stepper_info_list[motor_id][
            'target_position_callback'] = target_callback

        command = [PrivateConstants.STEPPER_GET_TARGET_POSITION, motor_id]
        await self.send_command(command)

    async def stepper_get_current_position(self, motor_id, current_position_callback):
        """
        Request the current motor position from the server.

        :param motor_id: 0 - 3

        :param current_position_callback: required callback function to receive report

        :return: The current motor position returned via the callback as a list:

        [REPORT_TYPE=17, motor_id, current position in steps, time_stamp]

        Positive is clockwise from the 0 position.
        """
        if not current_position_callback:
            if self.shutdown_on_exception:
                await self.shutdown()
            raise RuntimeError(
                'stepper_get_current_position Read: A callback function must be specified.')

        if not self.stepper_info_list[motor_id]['instance']:
            if self.shutdown_on_exception:
                await self.shutdown()
            raise RuntimeError('stepper_get_current_position: Invalid motor_id.')

        self.stepper_info_list[motor_id][
            'current_position_callback'] = current_position_callback

        command = [PrivateConstants.STEPPER_GET_CURRENT_POSITION, motor_id]
        await self.send_command(command)

    async def stepper_set_current_position(self, motor_id, position):
        """
        Resets the current position of the motor, so that wherever the motor
        happens to be right now is considered to be the new 0 position. Useful
        for setting a zero position on a stepper after an initial hardware
        positioning move.

        Has the side effect of setting the current motor speed to 0.

        :param motor_id:  0 - 3

        :param position: Position in steps. This is a 32 bit value
        """

        if not self.stepper_info_list[motor_id]['instance']:
            if self.shutdown_on_exception:
                await self.shutdown()
            raise RuntimeError('stepper_set_current_position: Invalid motor_id.')
        position_bytes = list(position.to_bytes(4, 'big', signed=True))

        command = [PrivateConstants.STEPPER_SET_CURRENT_POSITION, motor_id]
        for value in position_bytes:
            command.append(value)
        await self.send_command(command)

    async def stepper_run_speed_to_position(self, motor_id, completion_callback=None):
        """
        Runs the motor at the currently selected speed until the target position is
        reached.

        Does not implement accelerations.

        :param motor_id: 0 - 3

        :param completion_callback: call back function to receive motion complete
                                    notification

        callback returns a data list:

        [report_type, motor_id, raw_time_stamp]

        The report_type = 19
        """
        if not completion_callback:
            if self.shutdown_on_exception:
                await self.shutdown()
            raise RuntimeError('stepper_run_speed_to_position: A motion complete '
                               'callback must be '
                               'specified.')
        if not self.stepper_info_list[motor_id]['instance']:
            if self.shutdown_on_exception:
                await self.shutdown()
            raise RuntimeError('stepper_run_speed_to_position: Invalid motor_id.')

        self.stepper_info_list[motor_id]['motion_complete_callback'] = completion_callback
        command = [PrivateConstants.STEPPER_RUN_SPEED_TO_POSITION, motor_id]
        await self.send_command(command)

    async def stepper_stop(self, motor_id):
        """
        Sets a new target position that causes the stepper
        to stop as quickly as possible, using the current speed and
        acceleration parameters.

        :param motor_id:  0 - 3
        """
        if not self.stepper_info_list[motor_id]['instance']:
            if self.shutdown_on_exception:
                await self.shutdown()
            raise RuntimeError('stepper_stop: Invalid motor_id.')

        command = [PrivateConstants.STEPPER_STOP, motor_id]
        await self.send_command(command)

    async def stepper_disable_outputs(self, motor_id):
        """
        Disable motor pin outputs by setting them all LOW.

        Depending on the design of your electronics this may turn off
        the power to the motor coils, saving power.

        This is useful to support low power modes: disable the outputs
        during sleep and then re-enable with enableOutputs() before stepping
        again.

        If the enable Pin is defined, sets the pin to OUTPUT mode and clears
        the pin to be disabled.

        :param motor_id: 0 - 3
        """
        if not self.stepper_info_list[motor_id]['instance']:
            if self.shutdown_on_exception:
                await self.shutdown()
            raise RuntimeError('stepper_disable_outputs: Invalid motor_id.')

        command = [PrivateConstants.STEPPER_DISABLE_OUTPUTS, motor_id]
        await self.send_command(command)

    async def stepper_enable_outputs(self, motor_id):
        """
        Enable motor pin outputs by setting the motor pins to OUTPUT
        mode.

        If the enable Pin is defined, sets it to OUTPUT mode and sets
        the pin to be enabled.

        :param motor_id: 0 - 3
        """
        if not self.stepper_info_list[motor_id]['instance']:
            if self.shutdown_on_exception:
                await self.shutdown()
            raise RuntimeError('stepper_enable_outputs: Invalid motor_id.')

        command = [PrivateConstants.STEPPER_ENABLE_OUTPUTS, motor_id]
        await self.send_command(command)

    async def stepper_set_min_pulse_width(self, motor_id, minimum_width):
        """
        Sets the minimum pulse width allowed by the stepper driver.

        The minimum practical pulse width is approximately 20 microseconds.

        Times less than 20 microseconds will usually result in 20 microseconds or so.

        :param motor_id: 0 -3

        :param minimum_width: A 16 bit unsigned value expressed in microseconds.
        """
        if not self.stepper_info_list[motor_id]['instance']:
            if self.shutdown_on_exception:
                await self.shutdown()
            raise RuntimeError('stepper_set_min_pulse_width: Invalid motor_id.')

        if not 0 < minimum_width <= 0xff:
            if self.shutdown_on_exception:
                await self.shutdown()
            raise RuntimeError('stepper_set_min_pulse_width: Pulse width range = '
                               '0-0xffff.')

        width_msb = minimum_width >> 8
        width_lsb = minimum_width & 0xff

        command = [PrivateConstants.STEPPER_SET_MINIMUM_PULSE_WIDTH, motor_id, width_msb,
                   width_lsb]
        await self.send_command(command)

    async def stepper_set_enable_pin(self, motor_id, pin=0xff):
        """
        Sets the enable-pin number for stepper drivers.
        0xFF indicates unused (default).

        Otherwise, if a pin is set, the pin will be turned on when
        enableOutputs() is called and switched off when disableOutputs()
        is called.

        :param motor_id: 0 - 4
        :param pin: 0-0xff
        """
        if not self.stepper_info_list[motor_id]['instance']:
            if self.shutdown_on_exception:
                await self.shutdown()
            raise RuntimeError('stepper_set_enable_pin: Invalid motor_id.')

        if not 0 < pin <= 0xff:
            if self.shutdown_on_exception:
                await self.shutdown()
            raise RuntimeError('stepper_set_enable_pin: Pulse width range = '
                               '0-0xff.')
        command = [PrivateConstants.STEPPER_SET_ENABLE_PIN, motor_id, pin]

        await self.send_command(command)

    async def stepper_set_3_pins_inverted(self, motor_id, direction=False, step=False,
                                          enable=False):
        """
        Sets the inversion for stepper driver pins.

        :param motor_id: 0 - 3

        :param direction: True=inverted or False

        :param step: True=inverted or False

        :param enable: True=inverted or False
        """
        if not self.stepper_info_list[motor_id]['instance']:
            if self.shutdown_on_exception:
                await self.shutdown()
            raise RuntimeError('stepper_set_3_pins_inverted: Invalid motor_id.')

        command = [PrivateConstants.STEPPER_SET_3_PINS_INVERTED, motor_id, direction,
                   step, enable]

        await self.send_command(command)

    async def stepper_set_4_pins_inverted(self, motor_id, pin1_invert=False,
                                          pin2_invert=False,
                                          pin3_invert=False, pin4_invert=False,
                                          enable=False):
        """
        Sets the inversion for 2, 3 and 4 wire stepper pins

        :param motor_id: 0 - 3

        :param pin1_invert: True=inverted or False

        :param pin2_invert: True=inverted or False

        :param pin3_invert: True=inverted or False

        :param pin4_invert: True=inverted or False

        :param enable: True=inverted or False
        """
        if not self.stepper_info_list[motor_id]['instance']:
            if self.shutdown_on_exception:
                await self.shutdown()
            raise RuntimeError('stepper_set_4_pins_inverted: Invalid motor_id.')

        command = [PrivateConstants.STEPPER_SET_4_PINS_INVERTED, motor_id, pin1_invert,
                   pin2_invert, pin3_invert, pin4_invert, enable]

        await self.send_command(command)

    async def stepper_is_running(self, motor_id, callback):
        """
        Checks to see if the motor is currently running to a target.

        Callback return True if the speed is not zero or not at the target position.

        :param motor_id: 0-4

        :param callback: required callback function to receive report

        :return: The current running state returned via the callback as a list:

        [REPORT_TYPE=18, motor_id, True or False for running state, time_stamp]
        """
        if not callback:
            if self.shutdown_on_exception:
                await self.shutdown()
            raise RuntimeError(
                'stepper_is_running: A callback function must be specified.')

        if not self.stepper_info_list[motor_id]['instance']:
            if self.shutdown_on_exception:
                await self.shutdown()
            raise RuntimeError('stepper_is_running: Invalid motor_id.')

        self.stepper_info_list[motor_id]['is_running_callback'] = callback

        command = [PrivateConstants.STEPPER_IS_RUNNING, motor_id]
        await self.send_command(command)

    async def _set_pin_mode(self, pin_number, pin_state, differential=0, value_range=0,
                            callback=None):

        """
        A private method to set the various pin modes.

        :param pin_number: pico pin number

        :param pin_state: INPUT/OUTPUT/ANALOG/PWM

        :param differential: for analog inputs - threshold
                             value to be achieved for report to
                             be generated

                           : for servo we overload this variable to mean the minimum
                             duty cycle

        :param value_range: for servo this is the maximum duty cycle

        :param callback: A reference to a call back function to be
                         called when pin data value changes

        """
        # Map ADC to GPIO pin numbers
        if pin_state == PrivateConstants.AT_ANALOG:
            self.pico_pins[26 + pin_number] = PrivateConstants.AT_ANALOG
        else:
            if pin_number in self.pico_pins:
                self.pico_pins[pin_number] = pin_state
            else:
                raise RuntimeError('Gpio Pin Number is invalid')

        if callback:
            if pin_state == PrivateConstants.AT_INPUT:
                self.digital_callbacks[pin_number] = callback
            elif pin_state == PrivateConstants.AT_INPUT_PULLUP:
                self.digital_callbacks[pin_number] = callback
            elif pin_state == PrivateConstants.AT_INPUT_PULL_DOWN:
                self.digital_callbacks[pin_number] = callback
            elif pin_state == PrivateConstants.AT_ANALOG:
                self.analog_callbacks[pin_number] = callback

            else:
                print('{} {}'.format('set_pin_mode: callback ignored for '
                                     'pin state:', pin_state))

        if pin_state == PrivateConstants.AT_INPUT:
            command = [PrivateConstants.SET_PIN_MODE, pin_number,
                       PrivateConstants.AT_INPUT, 1]

        elif pin_state == PrivateConstants.AT_INPUT_PULLUP:
            command = [PrivateConstants.SET_PIN_MODE, pin_number,
                       PrivateConstants.AT_INPUT_PULLUP, 1]

        elif pin_state == PrivateConstants.AT_INPUT_PULL_DOWN:
            command = [PrivateConstants.SET_PIN_MODE, pin_number,
                       PrivateConstants.AT_INPUT_PULL_DOWN, 1]

        elif pin_state == PrivateConstants.AT_OUTPUT:
            command = [PrivateConstants.SET_PIN_MODE, pin_number,
                       PrivateConstants.AT_OUTPUT]

        elif pin_state == PrivateConstants.AT_ANALOG:
            df = differential.to_bytes(2, byteorder='big')

            # last parameter enable reporting
            command = [PrivateConstants.SET_PIN_MODE, pin_number,
                       PrivateConstants.AT_ANALOG, df[0], df[1], 1]

        elif pin_state == PrivateConstants.AT_PWM_OUTPUT:
            command = [PrivateConstants.SET_PIN_MODE, pin_number,
                       PrivateConstants.AT_OUTPUT]

        elif pin_state == PrivateConstants.AT_SERVO:
            # differential is being used for the min
            # value range is being used for the max value
            df = differential.to_bytes(2, byteorder='big')
            vr = value_range.to_bytes(2, byteorder='big')
            command = [PrivateConstants.SERVO_ATTACH, pin_number, df[0], df[1], vr[0],
                       vr[1]]

            self.servo_ranges[pin_number] = [differential, value_range]

        else:
            if self.shutdown_on_exception:
                await self.shutdown()
            raise RuntimeError('Unknown pin state')

        if pin_state == PrivateConstants.AT_ANALOG:
            if pin_number == 0:
                self.pico_pins[26] = PrivateConstants.AT_ANALOG
            elif pin_number == 1:
                self.pico_pins[27] = PrivateConstants.AT_ANALOG
            elif pin_number == 13:
                self.pico_pins[28] = PrivateConstants.AT_ANALOG

        else:
            self.pico_pins[pin_number] = pin_state

        if command:
            await self.send_command(command)

    async def shutdown(self):
        """
        This method attempts to perform an orderly shutdown.
        If any exceptions are thrown, they are ignored.
        """
        self.shutdown_flag = True

        try:
            command = [PrivateConstants.STOP_ALL_REPORTS]
            await self.send_command(command)
            time.sleep(.1)

            command = [PrivateConstants.RESET, self.reset_board_on_shutdown]
            await self.send_command(command)

            time.sleep(1)
            try:
                self.sock.shutdown(socket.SHUT_RDWR)
                self.sock.close()
            except Exception:
                pass

            self.the_task.cancel()
            await asyncio.sleep(.5)
            if self.close_loop_on_shutdown:
                self.loop.stop()

        except Exception:
            raise RuntimeError('Shutdown failed - could not send stop streaming message')

    '''
    report message handlers
    '''

    async def _analog_message(self, data):
        """
        This is a private message handler method.
        It is a message handler for analog messages.

        :param data: message data

        """
        pin = data[0]
        value = (data[1] << 8) + data[2]
        # set the current value in the pin structure
        time_stamp = time.time()
        # self.digital_pins[pin].event_time = time_stamp
        if self.analog_callbacks[pin]:
            message = [PrivateConstants.ANALOG_REPORT, pin, value, time_stamp]
            self.analog_callbacks[pin](message)

    async def _cpu_temp_message(self, data):
        """
        This is a private message handler method.
        It is a message handler for cpu temperature messages.

        :param data: message data

        """

        temperature = struct.unpack('<f', bytes(data))
        temperature = round(temperature[0], 2)
        time_stamp = time.time()
        # self.digital_pins[pin].event_time = time_stamp
        if self.cpu_temp_callback:
            message = [PrivateConstants.CPU_TEMP_REPORT, temperature, time_stamp]
            self.cpu_temp_callback(message)

    async def _dht_report(self, report):
        """
        This is a private utility method.

        This is the dht report handler method.

        :param report:
               data[0] = report error return
                                No Errors = 0

                                Checksum Error = 1

                                Timeout Error = 2

                                Invalid Value = 999

               data[1] = pin number

               data[2] = humidity positivity flag

               data[3] = temperature positivity value

               data[4] = humidity integer

               data[5] = humidity fractional value

               data[6] = temperature integer

               data[7] = temperature fractional value


                """
        if report[0]:  # DHT_ERROR
            # error report
            # data[0] = report sub type, data[1] = pin, data[2] = error message
            if self.dht_callbacks[report[1]]:
                # Callback 0=DHT REPORT, DHT_ERROR, PIN, Time
                message = [PrivateConstants.DHT_REPORT, report[0], report[1], report[2],
                           time.time()]
                self.dht_callbacks[report[1]](message)
        else:
            # got valid data DHT_DATA
            f_humidity = float(report[4] + report[5] / 100)
            if report[2]:
                f_humidity *= -1.0
            f_temperature = float(report[6] + report[7] / 100)
            if report[3]:
                f_temperature *= -1.0
            message = [PrivateConstants.DHT_REPORT, report[0], report[1], report[2],
                       f_humidity, f_temperature, time.time()]

            self.dht_callbacks[report[1]](message)

    async def _digital_message(self, data):
        """
        This is a private message handler method.
        It is a message handler for Digital Messages.

        :param data: digital message

        """
        pin = data[0]
        value = data[1]

        time_stamp = time.time()
        if self.digital_callbacks[pin]:
            message = [PrivateConstants.DIGITAL_REPORT, pin, value, time_stamp]
            self.digital_callbacks[pin](message)

    async def _firmware_message(self, data):
        """
        Telemetrix4pico firmware version message
        :param data: data[0] = major number, data[1] = minor number
        """

        self.firmware_version = [data[0], data[1]]

    async def _i2c_read_report(self, data):
        """
        Execute callback for i2c reads.

        :param data: [I2C_READ_REPORT, i2c_port, i2c_address,
        register, number of bytes read, bytes read..., time-stamp]
        """

        cb_list = [PrivateConstants.I2C_READ_REPORT, data[0], data[1]] + data[2:]

        cb_list.append(time.time())

        if cb_list[1]:
            self.i2c_callback2(cb_list)
        else:
            self.i2c_callback(cb_list)

    async def _i2c_too_few_bytes_received(self, data):
        """
        I2c write attempt failed

        :param data: data[0] = i2c_device
        """
        if self.shutdown_on_exception:
            await self.shutdown()
        raise RuntimeError(
            f'i2c Too few bytes received for I2C port {data[0]}')
        while True:
            time.sleep(1)

    async def _i2c_too_many_bytes_received(self, data):
        """
        I2c read failed

        :param data: data[0] = i2c device
        """
        if self.shutdown_on_exception:
            await self.shutdown()
        raise RuntimeError(
            f'i2c too many bytes received for I2C port {data[0]}')
        while True:
            time.sleep(.1)

    async def _report_unique_id(self, data):
        """
        Reply to are_u_there message
        :param data: pico id
        """

        for i in range(len(data)):
            self.reported_pico_id.append(data[i])

    async def _report_debug_data(self, data):
        """
        Print debug data sent from pico
        :param data: data[0] is a byte followed by 2
                     bytes that comprise an integer
        :return:
        """
        value = (data[1] << 8) + data[2]
        print(f'DEBUG ID: {data[0]} Value: {value}')

    async def _report_loop_data(self, data):
        """
        Print data that was looped back
        :param data: byte of loop back data
        :return:
        """
        if self.loop_back_callback:
            self.loop_back_callback(data)

    # onewire is not available for the pico
    # def _onewire_report(self, report):
    #     cb_list = [PrivateConstants.ONE_WIRE_REPORT, report[0]] + report[1:]
    #     cb_list.append(time.time())
    #     self.onewire_callback(cb_list)

    async def _stepper_distance_to_go_report(self, report):
        """
        Report stepper distance to go.

        :param report: data[0] = motor_id, data[1] = steps MSB, data[2] = steps byte 1,
                                 data[3] = steps bytes 2, data[4] = steps LSB

        callback report format: [PrivateConstants.STEPPER_DISTANCE_TO_GO, motor_id
                                 steps, time_stamp]
        """

        # get callback
        cb = self.stepper_info_list[report[0]]['distance_to_go_callback']

        # isolate the steps bytes and covert list to bytes
        steps = bytes(report[1:])

        # get value from steps
        num_steps = int.from_bytes(steps, byteorder='big', signed=True)

        cb_list = [PrivateConstants.STEPPER_DISTANCE_TO_GO, report[0], num_steps,
                   time.time()]

        cb(cb_list)

    async def _stepper_target_position_report(self, report):
        """
        Report stepper target position to go.

        :param report: data[0] = motor_id, data[1] = target position MSB,
                       data[2] = target position byte MSB+1
                       data[3] = target position byte MSB+2
                       data[4] = target position LSB

        callback report format: [PrivateConstants.STEPPER_TARGET_POSITION, motor_id
                                 target_position, time_stamp]
        """

        # get callback
        cb = self.stepper_info_list[report[0]]['target_position_callback']

        # isolate the steps bytes and covert list to bytes
        target = bytes(report[1:])

        # get value from steps
        target_position = int.from_bytes(target, byteorder='big', signed=True)

        cb_list = [PrivateConstants.STEPPER_TARGET_POSITION, report[0], target_position,
                   time.time()]

        cb(cb_list)

    async def _stepper_current_position_report(self, report):
        """
        Report stepper current position.

        :param report: data[0] = motor_id, data[1] = current position MSB,
                       data[2] = current position byte MSB+1
                       data[3] = current position byte MSB+2
                       data[4] = current position LSB

        callback report format: [PrivateConstants.STEPPER_CURRENT_POSITION, motor_id
                                 current_position, time_stamp]
        """

        # get callback
        cb = self.stepper_info_list[report[0]]['current_position_callback']

        # isolate the steps bytes and covert list to bytes
        position = bytes(report[1:])

        # get value from steps
        current_position = int.from_bytes(position, byteorder='big', signed=True)

        cb_list = [PrivateConstants.STEPPER_CURRENT_POSITION, report[0], current_position,
                   time.time()]

        cb(cb_list)

    async def _stepper_is_running_report(self, report):
        """
        Report if the motor is currently running

        :param report: data[0] = motor_id, True if motor is running or False if it is not.

        callback report format: [18, motor_id,
                                 running_state, time_stamp]
        """

        # get callback
        cb = self.stepper_info_list[report[0]]['is_running_callback']

        cb_list = [PrivateConstants.STEPPER_RUNNING_REPORT, report[0], time.time()]

        cb(cb_list)

    async def _stepper_run_complete_report(self, report):
        """
        The motor completed it motion

        :param report: data[0] = motor_id

        callback report format: [PrivateConstants.STEPPER_RUN_COMPLETE_REPORT, motor_id,
                                 time_stamp]
        """

        # get callback
        cb = self.stepper_info_list[report[0]]['motion_complete_callback']

        cb_list = [PrivateConstants.STEPPER_RUN_COMPLETE_REPORT, report[0],
                   time.time()]

        cb(cb_list)

    async def _features_report(self, report):
        self.reported_features = report[0]

    async def _send_command(self, command):
        """
        This is a private utility method.


        :param command:  command data in the form of a list

        """
        # the length of the list is added at the head
        command.insert(0, len(command))
        # print(command)
        send_message = bytes(command)

        self.sock.sendall(send_message)

    async def _servo_unavailable(self, report):
        """
        Message if no servos are available for use.
        :param report: pin number
        """
        if self.shutdown_on_exception:
            await self.shutdown()
        raise RuntimeError(
            f'Servo Attach For Pin {report[0]} Failed: No Available Servos')

    async def _sonar_distance_report(self, report):
        """

        :param report: data[0] = trigger pin, data[1] and data[2] = distance

        callback report format: [PrivateConstants.SONAR_DISTANCE, trigger_pin,
        distance  in centimeters, time_stamp]
        """

        # get callback from pin number
        cb = self.sonar_callbacks[report[0]]

        # build report data
        if report[1] == 0 and report[2] == 0:
            cb_list = [PrivateConstants.SONAR_DISTANCE, report[0],
                       0, time.time()]
        else:
            cb_list = [PrivateConstants.SONAR_DISTANCE, report[0],
                       (report[1] + (report[2] / 100)), time.time()]

        cb(cb_list)

    async def _spi_report(self, report):
        """
        Execute callback for spi reads.

        :param report: [spi_port, number of bytes read, data]

        """

        cb_list = [PrivateConstants.SPI_REPORT, report[0], report[1]] + report[2:]

        cb_list.append(time.time())

        if cb_list[1]:
            self.spi_callback2(cb_list)
        else:
            self.spi_callback(cb_list)

    async def _run_threads(self):
        self.run_event.set()

    async def _is_running(self):
        return self.run_event.is_set()

    async def _stop_threads(self):
        self.run_event.clear()

    async def _reporter(self):
        """
        This is the reporter thread. It continuously pulls data from
        the deque. When a full message is detected, that message is
        processed.
        """
        self.run_event.wait()

        while self._is_running() and not self.shutdown_flag:
            if len(self.the_deque):
                # response_data will be populated with the received data for the report
                response_data = []
                packet_length = self.the_deque.popleft()

                if packet_length:
                    # get all the data for the report and place it into response_data
                    for i in range(packet_length):
                        while not len(self.the_deque):
                            time.sleep(self.sleep_tune)
                        data = self.the_deque.popleft()
                        response_data.append(data)

                    # get the report type and look up its dispatch method
                    # here we pop the report type off of response_data
                    report_type = response_data.pop(0)

                    # retrieve the report handler from the dispatch table
                    dispatch_entry = self.report_dispatch.get(report_type)

                    # if there is additional data for the report,
                    # it will be contained in response_data
                    # noinspection PyArgumentList
                    await dispatch_entry(response_data)
                    continue

                else:
                    if self.shutdown_on_exception:
                        await self.shutdown()
                    raise RuntimeError(
                        'A report with a packet length of zero was received.')
            else:
                time.sleep(self.sleep_tune)

    async def _tcp_receiver(self):
        """
        This is a private utility method.

        Thread to continuously check for incoming data.
        When a byte comes in, place it onto the deque.
        """
        self.run_event.wait()

        # Start this thread only if ip_address is set

        if self.ip_address:

            while self._is_running() and not self.shutdown_flag:
                try:
                    payload = self.sock.recv(1)
                    self.the_deque.append(ord(payload))
                except Exception:
                    pass
        else:
            return

    async def _arduino_report_dispatcher(self):
        """
        This is a private method.
        It continually accepts and interprets data coming from Telemetrix4Arduino,and then
        dispatches the correct handler to process the data.

        It first receives the length of the packet, and then reads in the rest of the
        packet. A packet consists of a length, report identifier and then the report data.
        Using the report identifier, the report handler is fetched from report_dispatch.

        :returns: This method never returns
        """

        while True:
            if self.shutdown_flag:
                break
            try:
                packet_length = ord(await self.sock.read())
            except TypeError:
                continue

            # get the rest of the packet

            packet = list(await self.sock.read(packet_length))

            report = packet[0]
            # handle all other messages by looking them up in the
            # command dictionary

            # noinspection PyArgumentList
            await self.report_dispatch[report](packet[1:])
            await asyncio.sleep(self.sleep_tune)
