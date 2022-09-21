## Importing
To import this package, the standard import string is:

```angular2html
from telemetrix_rpi_pico_w_aio import telemetrix_rpi_pico_w_aio
```

Here is an example of instantiating the library and blinking the board LED. 

```python

import asyncio
import sys

from telemetrix_rpi_pico_w_aio import telemetrix_rpi_pico_w_aio

"""
Setup a pin for digital output and output a signal
and toggle the pin. Do this 4 times.
"""

# some globals
DIGITAL_PIN = 32  # Board LED pin number

async def blink(my_board, pin):
    """
    This function toggles a digital pin four times and then exits.
    :param my_board: telemetrix instance
    :param pin: pin to be controlled
    """

    # set the pin mode
    await my_board.set_pin_mode_digital_output(pin)

    # toggle the pin 4 times and exit
    for x in range(4):
        print('ON')
        await my_board.digital_write(pin, 0)
        await asyncio.sleep(1)
        print('OFF')
        await my_board.digital_write(pin, 1)
        await asyncio.sleep(1)


# get the event loop
loop = asyncio.new_event_loop()
asyncio.set_event_loop(loop)

# instantiate telemetrix_rpi_pico_w_aio
board = telemetrix_rpi_pico_w_aio.TelemetrixRpiPicoWAio(ip_address='192.168.2.102')

try:
    # start the main function
    loop.run_until_complete(blink(board, DIGITAL_PIN))
    loop.run_until_complete(board.shutdown())

except KeyboardInterrupt:
    loop.run_until_complete(board.shutdown())
    sys.exit(0)

```

To exit your 
application cleanly, you should always call the **_shutdown_** method.


## The \__init__ Method

For most applications, when instantiating the library, you can accept all the default 
parameters, but you must supply the IP address assigned to the Pico W server.

```python
board = telemetrix_rpi_pico_w_aio.TelemetrixRpiPicoWAio(ip_address='192.168.2.102')
```

However, several parameters offered by the \__init__ method allow for some additional 
control.

```python
def __init__(self, ip_address=None,
                 ip_port=31335,
                 sleep_tune=0.000001,
                 autostart=True,
                 loop=None,
                 shutdown_on_exception=True,
                 reset_on_shutdown=True,
                 close_loop_on_shutdown=True):

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
        :param close_loop_on_shutdown: If true, close the loop during shutdown
        """
```

Let's take a look at these parameters.
### ip_address

This mandatory parameter must be set to the IP address assigned to the Pico W server. 

### ip_port

The ip_port value must be the same for both the client and the Pico W server. 
Typically, this value is accepted as is.
If you change the value from the default, you must compile a matching value 
in the Pico W server.

### sleep_tune
This parameter is the sleep value expressed in seconds and is used at several strategic
points in the client. For example, the serial receiver continuously checks the serial 
port receive
buffer for an available
character to process. If there is no character in the
buffer, the client sleeps for the sleep_tune period before checking again.

Typically, you would accept the default value.

### autostart

Typically, you wish to have your program start all tasks automatically, 
however, you may delay startup by setting this parameter to False and add
your custom code before calling the start_aio method

### loop

You may specify an asyncio loop to use. The default is to allow the system
to use the default event loop.

### shutdown_on_exception
When this parameter is set to True, the library _shutdown_ method is automatically
called when an exception is detected and all reporting is disabled.

By setting this parameter to False, the Pico may continue to send data to
your application even after restarting it.

The default is True and is recommended to be used.

### reset_on_shutdown
When set to True (the default), this parameter will send a message to the Pico 
to reset itself. After resetting, the Pico board LED 
will flash twice, indicating the Pico has been reset.

### close_loop_on_shutdown
Typically the event loop is shutdown when the application exits. If you
wish to leave event loop open, set this parameter False.
   
<br>
<br>

Copyright (C) 2022 Alan Yorinks. All Rights Reserved.
