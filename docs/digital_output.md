## set_pin_mode_digital_output
```python
  def set_pin_mode_digital_output(self, pin_number)

    Set a pin as a digital output pin.

    :param pin_number: pico pin number
```

**NOTE:** This method must be called before calling digital_write.
## digital_write
```python
 def digital_write(self, pin, value)

    Set the specified pin to the specified value.

    :param pin: pico pin number

    :param value: pin value (1 or 0)
```
 
## Example: [blink.py](https://github.com/MrYsLab/telemetrix-rpi-pico-w/blob/master/examples/blink.py)


<br>
<br>

Copyright (C) 2022 Alan Yorinks. All Rights Reserved.
