## Importing
To import this package, the standard import string is:

```python
from telemetrix_rpi_pico_w import telemetrix_rpi_pico_w
```

Here is an example of instantiating the library and blinking the board LED 
until the user enters Control-C to end the application.


```python
import sys
import time
from telemetrix_rpi_pic_w import telemetrix_rpi_pico_w

# The GPIO pin number for the built-in LED
BOARD_LED = 32

# LED States
ON = 1
OFF = 0

# instantiate the library
# You must specify the IP address assigned to the Pico W server.

board = telemetrix_rpi_pico_w.TelemetrixRpiPicoW(ip_address='192.168.1.133')

# Set the DIGITAL_PIN as an output pin
board.set_pin_mode_digital_output(BOARD_LED)

try:
    while True:
        # turn led on
        board.digital_write(BOARD_LED, ON)
        time.sleep(1)
        # turn led off
        board.digital_write(BOARD_LED, OFF)
        time.sleep(1)

except KeyboardInterrupt:
    board.shutdown()
    sys.exit(0)

```

To exit your 
application cleanly, you should always call the **_shutdown_** method.


## The \__init__ Method

When instantiating the library, you can accept all the default parameters for most 
applications. However, you must specify the IP address assigned to the Pico W server.

```python
board = telemetrix_rpi_pico_w.TelemetrixRpiPicoW(ip_address='192.168.1.133')
```

Several parameters offered by the __init_\_ method allow for some additional 
control.

```python
def __init__(self, ip_address=None,
                 ip_port=31335,
                 sleep_tune=0.000001,
                 shutdown_on_exception=True,
                 reset_on_shutdown=True):

        """

        :param ip_address: IP address assigned to the Pico W

        :param ip_port: IP Port number.

        :param sleep_tune: A tuning parameter (typically not changed by user)

        :param shutdown_on_exception: call shutdown before raising
                                      a RunTimeError exception, or
                                      receiving a KeyboardInterrupt exception

        :param reset_on_shutdown: Reset the board upon shutdown
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

### shutdown_on_exception
When this parameter is set to True, the library _shutdown_ method is automatically
called when an exception is detected, and all reporting is disabled.

By setting this parameter to False, the Pico W may continue to send data to
your application even after restarting it.

The default is True and is recommended to be used.

### reset_on_shutdown
When set to True (the default), this parameter will send a message to the Pico W
to reset itself. After resetting, the Pico W board LED 
will stay lit until a connection is made with your router.


   
<br>
<br>

Copyright (C) 2022 Alan Yorinks. All Rights Reserved.
