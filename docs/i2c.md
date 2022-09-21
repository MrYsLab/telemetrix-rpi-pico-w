These methods are not specific to a given i2c device but instead allow you to control
any i2c device by implementing the device's protocol as specified by the 
manufacturer's datasheet.

## set_pin_mode_i2c
```python
  def set_pin_mode_i2c(self, i2c_port=0, sda_gpio=None, scl_gpio=None)

    Establish the standard pico i2c pins for i2c utilization.

    SDA pins: port 0 = 4 port 1 = 26 SCL pins: port 0 = 5 port 1 = 27

    :param i2c_port: 0 = i2c0, 1 = i2c1

    :param sda_gpio: gpio pin assigned to SDA

    :param scl_gpio: gpio pin assigned to SCL
    Notes

        THIS METHOD MUST BE CALLED BEFORE ANY I2C REQUEST IS MADE
        Callbacks are set within the individual i2c read methods of this API.

    See i2c_read, and i2c_write


```

This method must be called before calling either i2c_read or i2c_write. 
This method selects one of the i2c ports and associates the SDA and SCL pins with that port.

Note that GPIO pins 0 and 1 are not available for use.


## i2c_read

```python
 def i2c_read(self, address, register, number_of_bytes, callback=None, i2c_port=0, send_stop=True)

    Read the specified number of bytes from the specified register for the i2c device.

    :param address: i2c device address

    :param register: i2c register (or None if no register selection is needed)

    :param number_of_bytes: number of bytes to be read

    :param callback: Required callback function to report i2c data as a result of read command

    :param i2c_port: 0 = port 0, 1 = port 1

    :param send_stop: If False, master retains control of the bus at the end of the transfer (no Stop is issued), and the next transfer will begin with a Restart rather than a Start.

    callback returns a data list: [I2C_READ_REPORT, i2c_port, i2c_device_address, count of data bytes, data bytes, time-stamp]

    I2C_READ_REPORT = 10
```

This method allows you to read a specified number of bytes from the device. 

The **address** parameter specifies the i2c address of the device.

The **register** parameter specifies the i2c register to use. If the device does not 
require a register to be specified, this parameter is set to None.

The **number_of_bytes** parameter specifies how many bytes will be read from the device.
Data is returned via callback, and therefore you must specify a **callback** parameter.

The **i2c_port** specifies which of the two i2c ports to use for this device. The SDA 
and SCL pins are implied as a result of the call to  _set_pin_mode_i2c_.

Some devices require that after a read, the i2c master retain control of the bus. The 
**no_stop** parameter allows you to select this behavior.

The data returned to the callback is similar to all other callbacks. The items in the 
list passed to the callback function are:

[I2C_READ_REPORT, i2c_port, i2c_device_address, count of data bytes, data bytes, time-stamp]

The first element is the report type, and I2C_READ_REPORT has a value of 10. The 
i2c_port, the device's i2c address, the number of the bytes returned, the actual data 
bytes, and a time-stamp are also contained in the report.

## i2c_write

```python
  def i2c_write(self, address, args, i2c_port=0)

    Write data to an i2c device.

    :param address: i2c device address

    :param args: A variable number of bytes to be sent to the device passed in as a list. NOTE: THIS MUST BE IN THE FORM OF A LIST.

    :param i2c_port: 0= port 0, 1 = port 1

```
This method is used to write a variable number of bytes to an i2c device. You specify 
the i2c address, a list of the bytes to send, the i2c port to use.


## Example: [i2c_adxl345_accelerometer.py](https://github.com/MrYsLab/telemetrix-rpi-pico-w/blob/master/examples/i2c_adxl345_accelerometer.py)

## Example Sample Output:
```python


Raw Data:  [10, 0, 83, 50, 6, 248, 255, 7, 0, 97, 0]
ADXL345 Report On: 2021-05-04 17:36:52: 
		i2c_port=0 x-pair=248, 255  y-pair=7, 0 z-pair=97, 0
		
```

<br>
<br>

Copyright (C) 2022 Alan Yorinks. All Rights Reserved.
