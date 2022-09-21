## set_pin_mode_servo

```python
 async def set_pin_mode_servo(self, pin_number, min_pulse=1000, max_pulse=2000)

    Attach a pin to a servo motor

    Servo mode is a specialized version of PWM Output mode. There are 16 PWM pins shared between the Servo and PWM Output modes.

    :param pin_number: pin

    :param min_pulse: minimum pulse width in microseconds

    :param max_pulse: maximum pulse width in microseconds

```
**NOTE:** Servo-assigned pins are a special case of PWM,  so they share in the maximum 
assignment of 16 PWM pins.

## servo_write
```python
 async def servo_write(self, pin_number, value)

    Write the value to the specified servo

    :param pin_number: GPIO pin number

    :param value: value between 0 and 180 degrees
```


## Example: [servo_aio.py](https://github.com/MrYsLab/telemetrix-rpi-pico-w/blob/master/examples_aio/servo_aio.py)


<br>
<br>

Copyright (C) 2022 Alan Yorinks. All Rights Reserved.