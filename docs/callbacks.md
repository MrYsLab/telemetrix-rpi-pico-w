## Registering A Callback

A callback function must be registered when you set a pin to an input mode.

The code below illustrates a typical callback function. 

```angular2html
def the_callback(data):
    """
    A callback function to report data changes.
    This will print the pin number, its reported value and
    the date and time when the change occurred

    :param data: [pin mode, pin, current reported value, pin_mode, timestamp]
    """
    date = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(data[CB_TIME]))
    print(f'Report Type: {data[CB_PIN_MODE]} Pin: {data[CB_PIN]} '
          f'Value: {data[CB_VALUE]} Time Stamp: {date}')
```

And here, the callback is registered when the set_pin_mode is called:

```
board.set_pin_mode_digital_input_pullup(12, the_callback)
```

If you forget to register a callback, 
a RunTime exception will be thrown.

```angular2html
Traceback (most recent call last):
  File "/home/afy/PycharmProjects/telemetrix-rpi-pico-w/ play/no_callback_registered.py", line 5, in <module>
    board.set_pin_mode_digital_input(5)
  File "/home/afy/PycharmProjects/telemetrix-rpi-pico-w/telemetrix_rpi_pico_w/telemetrix_rpi_pico_w.py", line 752, in set_pin_mode_digital_input
    raise RuntimeError('A callback must be specified')
RuntimeError: A callback must be specified
```

## Callback Function Parameter

A callback function or method must accept a single parameter. The client automatically 
fills in this parameter in the form of a list when an input data change notification 
is received. For "the_callback" above, this parameter is named data.

The list contents vary from input pin type to input pin type and 
are described in detail for each _set_pin_mode_XXX_ method in the
[API documentation.](https://htmlpreview.github.io/?https://github.com/MrYsLab/telemetrix-rpi-pico/blob/master/html/telemetrix_rpi_pico/index.html) 
The first element in the list identifies the pin type, and the last element
is a timestamp of the data change occurrence. Other elements identify the GPIO pin, 
the current data value, and any additional relevant information.

For example, the list may contain

```angular2html
[DIGITAL_REPORT, pin_number, pin_value, raw_time_stamp]

DIGITAL_REPORT = 2
```

_**NOTE:**_

**Telemetrix does not support polling or direct read methods for inputs. 
Instead, as soon as a data change is detected, the pin's associated callback is 
called, allowing immediate response to data changes and, generally, 
a more straightforward application design.**



## Pin Types

|                  **Pin Mode**                   |  **Pin Mode Value**  |
|:-----------------------------------------------:|:--------------------:|
|  Digital Input (including pullup and pulldown)  |          2           |
|               Analog Input (ADC)                |          3           |
|                       I2C                       |          10          |
|                 SONAR Distance                  |          11          |
|                       DHT                       |          12          |


## Converting The Raw Timestamp

To convert the raw timestamp field to a human-readable time, use **time.localtime()**.

```angular2html
date = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(data[CB_TIME]))
```


<br>
<br>

Copyright (C) 2022 Alan Yorinks. All Rights Reserved.