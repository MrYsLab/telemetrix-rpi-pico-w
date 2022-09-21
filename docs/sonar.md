## set_pin_mode_sonar

```python
 def set_pin_mode_sonar(self, trigger_pin, echo_pin, callback=None)

    :param trigger_pin: Sensor trigger gpio pin

    :param echo_pin: Sensor echo gpio pin

    :param callback: callback

    callback returns a data list:

    [ SONAR_DISTANCE, trigger_pin, distance_value, time_stamp]

    SONAR_DISTANCE = 11
```

A maximum of  4 HC-SR04 type devices is supported. Reporting will immediately begin 
when this method is invoked.
<br>
<br>

## Example: [hc_sr04.py](https://github.com/MrYsLab/telemetrix-rpi-pico-w/blob/master/examples/hc_sr04.py)

## Example Sample Output (example modified for a single sensor):

```python
TelemetrixRpiPicoW:  Version 1.0

Copyright (c) 2022 Alan Yorinks All Rights Reserved.

2021-05-04 17:18:52	 Trigger Pin::	16	 Distance(cm):	32.06
2021-05-04 17:18:52	 Trigger Pin::	16	 Distance(cm):	32.06
2021-05-04 17:18:52	 Trigger Pin::	16	 Distance(cm):	31.17
2021-05-04 17:18:52	 Trigger Pin::	16	 Distance(cm):	30.22
```

Copyright (C) 2022 Alan Yorinks. All Rights Reserved.